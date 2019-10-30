#ifndef UNLOGGER_HPP
#define UNLOGGER_HPP

#include <QThread>
#include "messaging.hpp"
#include "FileReader.hpp"

class Unlogger : public QObject {
Q_OBJECT
  public:
    Unlogger(Events *events_);
    uint64_t getCurrentTime() { return t0; }
  public slots:
    void process();
  signals:
    void finished();
  private:
    Events *events;
    QMap<int, PubSocket*> socks;
    Context *ctx;
    uint64_t t0;
};

#endif

