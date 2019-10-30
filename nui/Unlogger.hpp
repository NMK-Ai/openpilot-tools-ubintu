#ifndef UNLOGGER_HPP
#define UNLOGGER_HPP

#include <QThread>
#include "messaging.hpp"
#include "FileReader.hpp"

class Unlogger : public QObject {
Q_OBJECT
  public:
    Unlogger(Events *events_);
    uint64_t getCurrentTime() { return tc; }
    void setSeekRequest(uint64_t seek_request_) { seek_request = seek_request_; }
  public slots:
    void process();
  signals:
    void elapsed();
    void finished();
  private:
    Events *events;
    QMap<int, PubSocket*> socks;
    Context *ctx;
    uint64_t tc = 0;
    uint64_t seek_request = 0;
};

#endif

