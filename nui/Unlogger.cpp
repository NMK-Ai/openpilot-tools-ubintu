#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "Unlogger.hpp"

Unlogger::Unlogger(Events *events_) : events(events_) {
  ctx = Context::create();
  YAML::Node service_list = YAML::LoadFile("/home/batman/one/selfdrive/service_list.yaml");
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

  t0 = (events->begin()+1).key();
  for (auto e : *events) {
    auto type = e.which();
    uint64_t tm = e.getLogMonoTime();
    auto it = socks.find(type);
    if (it != socks.end()) {
      float etime = (tm-t0)*1e-3;
      QThread::usleep(etime);

      capnp::MallocMessageBuilder msg;
      //auto ee = msg.initRoot<cereal::Event>();
      //ee = e.asBuilder();
      msg.setRoot(e);

      auto words = capnp::messageToFlatArray(msg);
      auto bytes = words.asBytes();

      // TODO: Can PubSocket take a const char?
      (*it)->send((char*)bytes.begin(), bytes.size());

      t0 = tm;
    }
  }
  emit finished();
}

