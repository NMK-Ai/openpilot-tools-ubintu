#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "Unlogger.hpp"

Unlogger::Unlogger(Events *events_) : events(events_) {
  ctx = Context::create();
  YAML::Node service_list = YAML::LoadFile("../..//selfdrive/service_list.yaml");
  for (const auto& it : service_list) {
    auto name = it.first.as<std::string>();
    PubSocket *sock = PubSocket::create(ctx, name);

    qDebug() << name.c_str();

    // TODO: is there a better way to do this?
    int type = -1;
    if (name == "controlsState") {
      type = cereal::Event::CONTROLS_STATE;
    } else if (name == "frame") {
      type = cereal::Event::FRAME;
    } else if (name == "sensorEvents") {
      type = cereal::Event::SENSOR_EVENTS;
    }

    if (type != -1) {
      socks.insert(type, sock);
    }
  }
}

void Unlogger::process() {
  qDebug() << "hello from unlogger thread";
  while (events->size() == 0) {
    qDebug() << "waiting for events";
    QThread::sleep(1);
  }
  qDebug() << "got events";

  QElapsedTimer timer;
  timer.start();
  uint64_t t0 = (events->begin()+1).key();

  for (auto e : *events) {
    auto type = e.which();
    uint64_t tm = e.getLogMonoTime();
    auto it = socks.find(type);
    tc = tm;
    if (it != socks.end()) {
      long etime = (tm-t0);
      long us_behind = ((etime-timer.nsecsElapsed())*1e-3)+0.5;
      if (us_behind > 0) {
        QThread::usleep(us_behind);
        //qDebug() << "sleeping" << us_behind << etime << timer.nsecsElapsed();
      }

      capnp::MallocMessageBuilder msg;
      msg.setRoot(e);
      auto words = capnp::messageToFlatArray(msg);
      auto bytes = words.asBytes();

      // TODO: Can PubSocket take a const char?
      (*it)->send((char*)bytes.begin(), bytes.size());
    }
  }
  emit finished();
}

