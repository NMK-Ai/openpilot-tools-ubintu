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

  uint64_t last_elapsed = 0;

  // loops
  while (1) {
    uint64_t t0 = (events->begin()+1).key();
    uint64_t t0r = timer.nsecsElapsed();
    qDebug() << "unlogging at" << t0;

    auto eit = events->lowerBound(t0);
    while (eit != events->end()) {
      if (seek_request != 0) {
        t0 = seek_request;
        qDebug() << "seeking to" << t0;
        t0r = timer.nsecsElapsed();
        eit = events->lowerBound(t0);
        seek_request = 0;
        if (eit == events->end()) {
          qWarning() << "seek off end";
          break;
        }
      }

      if (abs(tc-last_elapsed) > 50e6) {
        //qDebug() << "elapsed";
        emit elapsed();
        last_elapsed = tc;
      }

      auto e = *eit;
      auto type = e.which();
      uint64_t tm = e.getLogMonoTime();
      auto it = socks.find(type);
      tc = tm;
      if (it != socks.end()) {
        long etime = tm-t0;
        long rtime = timer.nsecsElapsed() - t0r;
        long us_behind = ((etime-rtime)*1e-3)+0.5;
        if (us_behind > 0) {
          if (us_behind > 1e6) {
            qWarning() << "OVER ONE SECOND BEHIND" << us_behind;
          }
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
      ++eit;
    }
  }
}

