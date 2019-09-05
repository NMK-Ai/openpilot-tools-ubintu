#!/usr/bin/env python
import argparse
import os
import sys
from collections import namedtuple

import cv2
import numpy as np
import pygame
import zmq

from common.basedir import BASEDIR
from common.transformations.camera import FULL_FRAME_SIZE, eon_intrinsics
from common.transformations.model import (MODEL_CX, MODEL_CY, MODEL_INPUT_SIZE,
                                          get_camera_frame_from_model_frame)
from selfdrive.car.toyota.interface import CarInterface as ToyotaInterface
from selfdrive.config import RADAR_TO_CENTER
from selfdrive.config import UIParams as UP
from selfdrive.controls.lib.lane_planner import (compute_path_pinv,
                                                 model_polyfit)
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.messaging import recv_one, recv_one_or_none, sub_sock
from selfdrive.services import service_list
from tools.lib.lazy_property import lazy_property
from tools.replay.lib.ui_helpers import (draw_lead_car, draw_lead_on, draw_mpc,
                                         draw_path, draw_steer_path,
                                         init_plots, to_lid_pt, warp_points)

os.environ['BASEDIR'] = BASEDIR

ANGLE_SCALE = 5.0
HOR = os.getenv("HORIZONTAL") is not None

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

_PATH_X = np.arange(192.)
_PATH_XD = np.arange(192.)
_PATH_PINV = compute_path_pinv(50)
#_BB_OFFSET = 290, 332
_BB_OFFSET = 0,0
_BB_SCALE = 1164/640.
_BB_TO_FULL_FRAME = np.asarray([
    [_BB_SCALE, 0., _BB_OFFSET[0]],
    [0., _BB_SCALE, _BB_OFFSET[1]],
    [0., 0.,   1.]])
_FULL_FRAME_TO_BB = np.linalg.inv(_BB_TO_FULL_FRAME)

ModelUIData = namedtuple("ModelUIData", ["cpath", "lpath", "rpath", "lead", "lead_future"])



class CalibrationTransformsForWarpMatrix(object):
  def __init__(self, model_to_full_frame, K, E):
    self._model_to_full_frame = model_to_full_frame
    self._K = K
    self._E = E

  @property
  def model_to_bb(self):
    return _FULL_FRAME_TO_BB.dot(self._model_to_full_frame)

  @lazy_property
  def model_to_full_frame(self):
    return self._model_to_full_frame

  @lazy_property
  def car_to_model(self):
    return np.linalg.inv(self._model_to_full_frame).dot(self._K).dot(
      self._E[:, [0, 1, 3]])

  @lazy_property
  def car_to_bb(self):
    return _BB_TO_FULL_FRAME.dot(self._K).dot(self._E[:, [0, 1, 3]])


def pygame_modules_have_loaded():
  return pygame.display.get_init() and pygame.font.get_init()

def draw_var(y, x, var, color, img, calibration, top_down):
  # otherwise drawing gets stupid
  var = max(1e-1, min(var, 0.7))

  varcolor = tuple(np.array(color)*0.5)
  draw_path(y - var, x, varcolor, img, calibration, top_down)
  draw_path(y + var, x, varcolor, img, calibration, top_down)


class ModelPoly(object):
  def __init__(self, model_path):
    if len(model_path.points) == 0 and len(model_path.poly) == 0:
      self.valid = False
      return

    if len(model_path.poly):
      self.poly = np.array(model_path.poly)
    else:
      self.poly = model_polyfit(model_path.points, _PATH_PINV)

    self.prob = model_path.prob
    self.std = model_path.std
    self.y = np.polyval(self.poly, _PATH_XD)
    self.valid = True

def extract_model_data(md):
  return ModelUIData(
    cpath=ModelPoly(md.model.path),
    lpath=ModelPoly(md.model.leftLane),
    rpath=ModelPoly(md.model.rightLane),
    lead=md.model.lead,
    lead_future=md.model.leadFuture,
    )

def plot_model(m, VM, v_ego, curvature, imgw, calibration, top_down, d_poly, top_down_color=216):
  if calibration is None or top_down is None:
    return

  for lead in [m.lead, m.lead_future]:
    if lead.prob < 0.5:
      continue

    _, py_top = to_lid_pt(lead.dist + lead.std, lead.relY)
    px, py_bottom = to_lid_pt(lead.dist - lead.std, lead.relY)
    top_down[1][int(round(px - 4)):int(round(px + 4)), py_top:py_bottom] = top_down_color

  color = (0, int(255 * m.lpath.prob), 0)
  for path in [m.cpath, m.lpath, m.rpath]:
    if path.valid:
      draw_path(path.y, _PATH_XD, color, imgw, calibration, top_down, YELLOW)
      draw_var(path.y, _PATH_XD, path.std, color, imgw, calibration, top_down)

  if d_poly is not None:
    dpath_y = np.polyval(d_poly, _PATH_X)
    draw_path(dpath_y, _PATH_X, RED, imgw, calibration, top_down, RED)

  # draw user path from curvature
  draw_steer_path(v_ego, curvature, BLUE, imgw, calibration, top_down, VM, BLUE)


def maybe_update_radar_points(lt, lid_overlay):
  ar_pts = []
  if lt is not None:
    ar_pts = {}
    for track in lt.liveTracks:
      ar_pts[track.trackId] = [track.dRel + RADAR_TO_CENTER, track.yRel, track.vRel, track.aRel, track.oncoming, track.stationary]
  for ids, pt in ar_pts.viewitems():
    px, py = to_lid_pt(pt[0], pt[1])
    if px != -1:
      if pt[-1]:
        color = 240
      elif pt[-2]:
        color = 230
      else:
        color = 255
      if int(ids) == 1:
        lid_overlay[px - 2:px + 2, py - 10:py + 10] = 100
      else:
        lid_overlay[px - 2:px + 2, py - 2:py + 2] = color

def get_blank_lid_overlay(UP):
  lid_overlay = np.zeros((UP.lidar_x, UP.lidar_y), 'uint8')
  # Draw the car.
  lid_overlay[int(round(UP.lidar_car_x - UP.car_hwidth)):int(
    round(UP.lidar_car_x + UP.car_hwidth)), int(round(UP.lidar_car_y -
                                                      UP.car_front))] = UP.car_color
  lid_overlay[int(round(UP.lidar_car_x - UP.car_hwidth)):int(
    round(UP.lidar_car_x + UP.car_hwidth)), int(round(UP.lidar_car_y +
                                                      UP.car_back))] = UP.car_color
  lid_overlay[int(round(UP.lidar_car_x - UP.car_hwidth)), int(
    round(UP.lidar_car_y - UP.car_front)):int(round(
      UP.lidar_car_y + UP.car_back))] = UP.car_color
  lid_overlay[int(round(UP.lidar_car_x + UP.car_hwidth)), int(
    round(UP.lidar_car_y - UP.car_front)):int(round(
      UP.lidar_car_y + UP.car_back))] = UP.car_color
  return lid_overlay


def ui_thread(addr, frame_address):
  context = zmq.Context.instance()

  # TODO: Detect car from replay and use that to select carparams
  CP = ToyotaInterface.get_params("TOYOTA PRIUS 2017", {})
  VM = VehicleModel(CP)

  CalP = np.asarray([[0, 0], [MODEL_INPUT_SIZE[0], 0], [MODEL_INPUT_SIZE[0], MODEL_INPUT_SIZE[1]], [0, MODEL_INPUT_SIZE[1]]])
  vanishing_point = np.asarray([[MODEL_CX, MODEL_CY]])

  pygame.init()
  pygame.font.init()
  assert pygame_modules_have_loaded()

  if HOR:
    size = (640+384+640, 960)
    write_x = 5
    write_y = 680
  else:
    size = (640+384, 960+300)
    write_x = 645
    write_y = 970

  pygame.display.set_caption("openpilot debug UI")
  screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)

  alert1_font = pygame.font.SysFont("arial", 30)
  alert2_font = pygame.font.SysFont("arial", 20)
  info_font = pygame.font.SysFont("arial", 15)

  camera_surface = pygame.surface.Surface((640, 480), 0, 24).convert()
  cameraw_surface = pygame.surface.Surface(MODEL_INPUT_SIZE, 0, 24).convert()
  cameraw_test_surface = pygame.surface.Surface(MODEL_INPUT_SIZE, 0, 24)
  top_down_surface = pygame.surface.Surface((UP.lidar_x, UP.lidar_y),0,8)

  frame = context.socket(zmq.SUB)
  frame.connect(frame_address or "tcp://%s:%d" % (addr, service_list['frame'].port))
  frame.setsockopt(zmq.SUBSCRIBE, "")

  carState = sub_sock(service_list['carState'].port, addr=addr, conflate=True)
  plan = sub_sock(service_list['plan'].port, addr=addr, conflate=True)
  carControl = sub_sock(service_list['carControl'].port, addr=addr, conflate=True)
  radar_state_sock = sub_sock(service_list['radarState'].port, addr=addr, conflate=True)
  liveCalibration = sub_sock(service_list['liveCalibration'].port, addr=addr, conflate=True)
  controls_state_sock = sub_sock(service_list['controlsState'].port, addr=addr, conflate=True)
  liveTracks = sub_sock(service_list['liveTracks'].port, addr=addr, conflate=True)
  model = sub_sock(service_list['model'].port, addr=addr, conflate=True)
  test_model = sub_sock(8040, addr=addr, conflate=True)
  liveMpc = sub_sock(service_list['liveMpc'].port, addr=addr, conflate=True)
  liveParameters = sub_sock(service_list['liveParameters'].port, addr=addr, conflate=True)
  pathPlan = sub_sock(service_list['pathPlan'].port, addr=addr, conflate=True)

  v_ego, angle_steers, angle_steers_des = 0., 0., 0.
  params_ao, params_ao_average, params_stiffness, params_sr = None, None, None, None

  enabled = False

  gas = 0.
  accel_override = 0.
  computer_gas = 0.
  brake = 0.
  steer_torque = 0.
  angle_steers_k = np.inf
  curvature = 0.
  computer_brake = 0.
  plan_source = 'none'
  long_control_state = 'none'
  d_poly = None

  model_data = None
  test_model_data = None
  a_ego = 0.0
  a_target = 0.0

  d_rel, y_rel, lead_status  = 0., 0., False
  d_rel2, y_rel2, lead_status2 = 0., 0., False

  v_ego, v_pid, v_cruise, v_override = 0., 0., 0., 0.
  brake_lights = False

  alert_text1, alert_text2 = "", ""

  intrinsic_matrix = None

  calibration = None
  #img = np.zeros((FULL_FRAME_SIZE[1], FULL_FRAME_SIZE[0], 3), dtype='uint8')
  img = np.zeros((480, 640, 3), dtype='uint8')
  imgff = np.zeros((FULL_FRAME_SIZE[1], FULL_FRAME_SIZE[0], 3), dtype=np.uint8)
  imgw = np.zeros((160, 320, 3), dtype=np.uint8)  # warped image
  good_lt = None
  lid_overlay_blank = get_blank_lid_overlay(UP)
  img_offset = (0, 0)

  # plots
  name_to_arr_idx = { "gas": 0,
                      "computer_gas": 1,
                      "user_brake": 2,
                      "computer_brake": 3,
                      "v_ego": 4,
                      "v_pid": 5,
                      "angle_steers_des": 6,
                      "angle_steers": 7,
                      "angle_steers_k": 8,
                      "steer_torque": 9,
                      "v_override": 10,
                      "v_cruise": 11,
                      "a_ego": 12,
                      "a_target": 13,
                      "accel_override": 14}

  plot_arr = np.zeros((100, len(name_to_arr_idx.values())))

  plot_xlims = [(0, plot_arr.shape[0]), (0, plot_arr.shape[0]), (0, plot_arr.shape[0]), (0, plot_arr.shape[0])]
  plot_ylims = [(-0.1, 1.1), (-ANGLE_SCALE, ANGLE_SCALE), (0., 75.), (-3.0, 2.0)]
  plot_names = [["gas", "computer_gas", "user_brake", "computer_brake", "accel_override"],
                ["angle_steers", "angle_steers_des", "angle_steers_k", "steer_torque"],
                ["v_ego", "v_override", "v_pid", "v_cruise"],
                ["a_ego", "a_target"]]
  plot_colors = [["b", "b", "g", "r", "y"],
                 ["b", "g", "y", "r"],
                 ["b", "g", "r", "y"],
                 ["b", "r"]]
  plot_styles = [["-", "-", "-", "-", "-"],
                 ["-", "-", "-", "-"],
                 ["-", "-", "-", "-"],
                 ["-", "-"]]

  draw_plots = init_plots(plot_arr, name_to_arr_idx, plot_xlims, plot_ylims, plot_names, plot_colors, plot_styles, bigplots=True)

  counter = 0
  while 1:
    list(pygame.event.get())

    screen.fill((64,64,64))
    lid_overlay = lid_overlay_blank.copy()
    top_down = top_down_surface, lid_overlay

    # ***** frame *****
    fpkt = recv_one(frame)
    rgb_img_raw = fpkt.frame.image

    if fpkt.frame.transform:
      img_transform = np.array(fpkt.frame.transform).reshape(3,3)
    else:
      # assume frame is flipped
      img_transform = np.array([
        [-1.0,  0.0, FULL_FRAME_SIZE[0]-1],
        [ 0.0, -1.0, FULL_FRAME_SIZE[1]-1],
        [ 0.0,  0.0, 1.0]
      ])


    if rgb_img_raw and len(rgb_img_raw) == FULL_FRAME_SIZE[0] * FULL_FRAME_SIZE[1] * 3:
      imgff = np.frombuffer(rgb_img_raw, dtype=np.uint8).reshape((FULL_FRAME_SIZE[1], FULL_FRAME_SIZE[0], 3))
      cv2.warpAffine(imgff, np.dot(img_transform, _BB_TO_FULL_FRAME)[:2],
        (img.shape[1], img.shape[0]), dst=img, flags=cv2.WARP_INVERSE_MAP)

      intrinsic_matrix = eon_intrinsics
    else:
      img.fill(0)
      intrinsic_matrix = np.eye(3)

    if calibration is not None:
      transform = np.dot(img_transform, calibration.model_to_full_frame)
      imgw = cv2.warpAffine(imgff, transform[:2], (MODEL_INPUT_SIZE[0], MODEL_INPUT_SIZE[1]), flags=cv2.WARP_INVERSE_MAP)
    else:
      imgw.fill(0)
    imgw_test_model = imgw.copy()


    # ***** controlsState *****
    controls_state = recv_one_or_none(controls_state_sock)
    if controls_state is not None:
      v_ego = controls_state.controlsState.vEgo
      angle_steers = controls_state.controlsState.angleSteers

      w = controls_state.controlsState.lateralControlState.which()
      if w == 'lqrState':
        angle_steers_k = controls_state.controlsState.lateralControlState.lqrState.steerAngle
      elif w == 'indiState':
        angle_steers_k = controls_state.controlsState.lateralControlState.indiState.steerAngle

      curvature = controls_state.controlsState.curvature
      v_pid = controls_state.controlsState.vPid
      enabled = controls_state.controlsState.enabled
      alert_text1 = controls_state.controlsState.alertText1
      alert_text2 = controls_state.controlsState.alertText2
      long_control_state = controls_state.controlsState.longControlState

    cs = recv_one_or_none(carState)
    if cs is not None:
      gas = cs.carState.gas
      brake_lights = cs.carState.brakeLights
      a_ego = cs.carState.aEgo
      brake = cs.carState.brake
      v_cruise = cs.carState.cruiseState.speed

    cc = recv_one_or_none(carControl)
    if cc is not None:
      v_override = cc.carControl.cruiseControl.speedOverride
      computer_brake = cc.carControl.actuators.brake
      computer_gas = cc.carControl.actuators.gas
      steer_torque = cc.carControl.actuators.steer * ANGLE_SCALE
      angle_steers_des = cc.carControl.actuators.steerAngle
      accel_override = cc.carControl.cruiseControl.accelOverride

    p = recv_one_or_none(plan)
    if p is not None:
      a_target = p.plan.aTarget
      plan_source = p.plan.longitudinalPlanSource

    pp = recv_one_or_none(pathPlan)
    if pp is not None:
      d_poly = np.array(pp.pathPlan.dPoly)

    plot_arr[:-1] = plot_arr[1:]
    plot_arr[-1, name_to_arr_idx['angle_steers']] = angle_steers
    plot_arr[-1, name_to_arr_idx['angle_steers_des']] = angle_steers_des

    plot_arr[-1, name_to_arr_idx['angle_steers_k']] = angle_steers_k
    plot_arr[-1, name_to_arr_idx['gas']] = gas
    plot_arr[-1, name_to_arr_idx['computer_gas']] = computer_gas
    plot_arr[-1, name_to_arr_idx['user_brake']] = brake
    plot_arr[-1, name_to_arr_idx['steer_torque']] = steer_torque
    plot_arr[-1, name_to_arr_idx['computer_brake']] = computer_brake
    plot_arr[-1, name_to_arr_idx['v_ego']] = v_ego
    plot_arr[-1, name_to_arr_idx['v_pid']] = v_pid
    plot_arr[-1, name_to_arr_idx['v_override']] = v_override
    plot_arr[-1, name_to_arr_idx['v_cruise']] = v_cruise
    plot_arr[-1, name_to_arr_idx['a_ego']] = a_ego
    plot_arr[-1, name_to_arr_idx['a_target']] = a_target
    plot_arr[-1, name_to_arr_idx['accel_override']] = accel_override

    # ***** model ****

    # live model
    md = recv_one_or_none(model)
    if md:
      model_data = extract_model_data(md)

    if model_data:
      plot_model(model_data, VM, v_ego, curvature, imgw, calibration,
                 top_down, d_poly)

    if test_model is not None:
      test_md = recv_one_or_none(test_model)
      if test_md:
        test_model_data = extract_model_data(test_md)

    if test_model_data:
      plot_model(test_model_data, VM, v_ego, curvature, imgw_test_model,
                 calibration, top_down, 215)

    # MPC
    mpc = recv_one_or_none(liveMpc)
    if mpc:
      draw_mpc(mpc, top_down)

    # LiveParams
    params = recv_one_or_none(liveParameters)
    if params:
      params_ao = params.liveParameters.angleOffset
      params_ao_average = params.liveParameters.angleOffsetAverage
      params_stiffness = params.liveParameters.stiffnessFactor
      params_sr = params.liveParameters.steerRatio

    # **** tracks *****

    # draw all radar points
    lt = recv_one_or_none(liveTracks)
    if lt is not None:
      good_lt = lt
    if good_lt is not None:
      maybe_update_radar_points(good_lt, top_down[1])

    # ***** radarState *****

    radar_state = recv_one_or_none(radar_state_sock)
    if radar_state is not None:
      d_rel = radar_state.radarState.leadOne.dRel + RADAR_TO_CENTER
      y_rel = radar_state.radarState.leadOne.yRel
      lead_status = radar_state.radarState.leadOne.status
      d_rel2 = radar_state.radarState.leadTwo.dRel + RADAR_TO_CENTER
      y_rel2 = radar_state.radarState.leadTwo.yRel
      lead_status2 = radar_state.radarState.leadTwo.status

    lcal = recv_one_or_none(liveCalibration)
    if lcal is not None:
      calibration_message = lcal.liveCalibration
      extrinsic_matrix = np.asarray(calibration_message.extrinsicMatrix).reshape(3, 4)

      ke = intrinsic_matrix.dot(extrinsic_matrix)
      warp_matrix = get_camera_frame_from_model_frame(ke)

      calibration = CalibrationTransformsForWarpMatrix(warp_matrix, intrinsic_matrix, extrinsic_matrix)

    # draw red pt for lead car in the main img
    if lead_status:
      if calibration is not None:
        dx, dy = draw_lead_on(img, d_rel, y_rel, img_offset, calibration, color=(192,0,0))
      # draw red line for lead car
      draw_lead_car(d_rel, top_down)

    # draw red pt for lead car2 in the main img
    if lead_status2:
      if calibration is not None:
        dx2, dy2 = draw_lead_on(img, d_rel2, y_rel2, img_offset, calibration, color=(192,0,0))
      # draw red line for lead car
      draw_lead_car(d_rel2, top_down)

    # *** blits ***
    pygame.surfarray.blit_array(camera_surface, img.swapaxes(0,1))
    screen.blit(camera_surface, (0, 0))

    # display alerts
    alert_line1 = alert1_font.render(alert_text1, True, (255,0,0))
    alert_line2 = alert2_font.render(alert_text2, True, (255,0,0))
    screen.blit(alert_line1, (180, 150))
    screen.blit(alert_line2, (180, 190))

    if calibration is not None and img is not None:
      cpw = warp_points(CalP, calibration.model_to_bb)
      vanishing_pointw = warp_points(vanishing_point, calibration.model_to_bb)
      pygame.draw.polygon(screen, BLUE, tuple(map(tuple, cpw)), 1)
      pygame.draw.circle(screen, BLUE, map(int, map(round, vanishing_pointw[0])), 2)

    if HOR:
      screen.blit(draw_plots(plot_arr), (640+384, 0))
    else:
      screen.blit(draw_plots(plot_arr), (0, 600))

    pygame.surfarray.blit_array(cameraw_surface, imgw.swapaxes(0, 1))
    screen.blit(cameraw_surface, (320, 480))

    pygame.surfarray.blit_array(cameraw_test_surface, imgw_test_model.swapaxes(0, 1))
    screen.blit(cameraw_test_surface, (0, 480))

    pygame.surfarray.blit_array(*top_down)
    screen.blit(top_down[0], (640,0))

    i = 0
    SPACING = 25

    # enabled
    enabled_line = info_font.render("ENABLED", True, GREEN if enabled else BLACK)
    screen.blit(enabled_line, (write_x, write_y + i * SPACING))
    i += 1

    # brake lights
    brake_lights_line = info_font.render("BRAKE LIGHTS", True, RED if brake_lights else BLACK)
    screen.blit(brake_lights_line, (write_x, write_y + i * SPACING))
    i += 1

    # speed
    v_ego_line = info_font.render("SPEED: " + str(round(v_ego, 1)) + " m/s", True, YELLOW)
    screen.blit(v_ego_line, (write_x, write_y + i * SPACING))
    i += 1

    # long control state
    long_control_state_line = info_font.render("LONG CONTROL STATE: " + str(long_control_state), True, YELLOW)
    screen.blit(long_control_state_line, (write_x, write_y + i * SPACING))
    i += 1

    # long mpc source
    plan_source_line = info_font.render("LONG MPC SOURCE: " + str(plan_source), True, YELLOW)
    screen.blit(plan_source_line, (write_x, write_y + i * SPACING))
    i += 1

    if params_ao is not None:
      i += 1
      angle_offset_avg_line = info_font.render("ANGLE OFFSET (AVG): " + str(round(params_ao_average, 2)) + " deg", True, YELLOW)
      screen.blit(angle_offset_avg_line, (write_x, write_y + i * SPACING))
      i += 1

      angle_offset_line = info_font.render("ANGLE OFFSET (INSTANT): " + str(round(params_ao, 2)) + " deg", True, YELLOW)
      screen.blit(angle_offset_line, (write_x, write_y + i * SPACING))
      i += 1

      angle_offset_line = info_font.render("STIFFNESS: " + str(round(params_stiffness * 100., 2)) + " %", True, YELLOW)
      screen.blit(angle_offset_line, (write_x, write_y + i * SPACING))
      i += 1

      steer_ratio_line = info_font.render("STEER RATIO: " + str(round(params_sr, 2)), True, YELLOW)
      screen.blit(steer_ratio_line, (write_x, write_y + i * SPACING))
      i += 1

    # this takes time...vsync or something
    pygame.display.flip()

def get_arg_parser():
  parser = argparse.ArgumentParser(
    description="Show replay data in a UI.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("ip_address", nargs="?", default="127.0.0.1",
                      help="The ip address on which to receive zmq messages.")

  parser.add_argument("--frame-address", default=None,
                      help="The frame address (fully qualified ZMQ endpoint for frames) on which to receive zmq messages.")
  return parser

if __name__ == "__main__":
  args = get_arg_parser().parse_args(sys.argv[1:])
  ui_thread(args.ip_address, args.frame_address)
