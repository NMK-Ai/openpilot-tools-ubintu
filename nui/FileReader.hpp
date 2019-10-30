#ifndef FILEREADER_HPP
#define FILEREADER_HPP

#include <QString>
#include <QNetworkAccessManager>
#include <QWidget>
#include <QVector>
#include <QMultiMap>
#include <QElapsedTimer>

#include <bzlib.h>

#include <kj/io.h>
#include <capnp/serialize.h>

#include "cereal/gen/cpp/log.capnp.h"

class FileReader : public QWidget {
public:
  FileReader(const QString& file);
  void startRequest(const QUrl &url);
  ~FileReader();
  virtual void readyRead();
  void httpFinished();
  virtual void done() {};
protected:
  QNetworkReply *reply;
private:
  QNetworkAccessManager qnam;
  QElapsedTimer timer;
};

typedef QMultiMap<uint64_t, cereal::Event::Reader> Events;

class LogReader : public FileReader {
public:
  LogReader(const QString& file, Events *);
  void readyRead();
  void done();

private:
  bz_stream bStream;

  // backing store
  QByteArray raw;
  int event_offset;
  Events *events;
};

class FrameReader : public FileReader {
public:
  FrameReader(const QString& file);
  void readyRead();
private:
};

#endif

