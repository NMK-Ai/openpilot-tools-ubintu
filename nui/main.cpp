#include <QApplication>
#include <QWidget>
#include <QString>
#include <QTimer>
#include <QPushButton>
#include <QGraphicsScene>
#include <QPainter>
#include <QThread>

#include "FileReader.hpp"
#include "Unlogger.hpp"

/*class Scrollbar : public QWidget {
  public:
    Scrollbar() {}
  protected:
    void paintEvent(QPaintEvent *event) override;
};

void Scrollbar::paintEvent(QPaintEvent *event) {
  printf("paint\n");
  //if (events.size() == 0) return;

  //QPainter painter(this);
}*/

// TODO: This is not thread safe
Events events;

class Window : public QWidget {
  public:
    Window();
  protected:
    void paintEvent(QPaintEvent *event) override;
    uint64_t ct;
    Unlogger *worker;
};

Window::Window() {
  //QPushButton *quitBtn = new QPushButton("Quit", this);
  //quitBtn->setGeometry(50, 40, 75, 30);

  //Scrollbar *sb = new Scrollbar();
  //sb->setGeometry(0, 0, 200, 100);

  QThread* thread = new QThread;
  worker = new Unlogger(&events);
  worker->moveToThread(thread);
  connect(thread, SIGNAL (started()), worker, SLOT (process()));
  thread->start();
}

void Window::paintEvent(QPaintEvent *event) {
  QElapsedTimer timer;
  timer.start();

  uint64_t t0 = events.begin().key();
  uint64_t t1 = (events.end()-1).key();

  QPainter p(this);

  p.setBrush(Qt::cyan);
  //p.drawRect(0, 0, 600, 100);

  int lt = -1;
  int lvv = 0;
  for (auto e : events) {
    auto type = e.which();
    //printf("%lld %d\n", e.getLogMonoTime()-t0, type);
    if (type == cereal::Event::CONTROLS_STATE) {
      auto controlsState = e.getControlsState();
      float t = (e.getLogMonoTime()-t0)*1e-9;
      float vEgo = controlsState.getVEgo();
      int enabled = controlsState.getState() == cereal::ControlsState::OpenpilotState::ENABLED;
      int rt = int(t*4.0+0.5); // 250 ms per pixel
      if (rt != lt) {
        int vv = vEgo*10.0;
        if (lt != -1) {
          p.setPen(Qt::red);
          p.drawLine(lt, 300-lvv, rt, 300-vv);

          if (enabled) {
            p.setPen(Qt::green); 
          } else {
            p.setPen(Qt::blue); 
          }

          p.drawLine(rt, 300, rt, 600);
        }
        lt = rt;
        lvv = vv;
      }

      //printf("%f : %f\n", t, vEgo);
    }
  }
  int rrt = int((worker->getCurrentTime()-t0)*1e-9*4.0+0.5);
  p.drawRect(rrt-1, 0, 2, 600);

  p.end();

  qDebug() << "paint in" << timer.elapsed() << "ms";
}

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);

  QString route(argv[1]);
  //route = "0006c839f32a6f99/2019-02-18--06-21-29";
  //route = "02ec6bea180a4d36/2019-10-25--10-18-09";
  route = "3a5d6ac1c23e5536/2019-10-29--10-06-58";

  Window window;

  QVector<LogReader*> lrs;
  //for (int i = 0; i <= 6; i++) {
  for (int i = 2; i <= 2; i++) {
    QString fn = QString("%1/%2/rlog.bz2").arg(route).arg(i);
    LogReader *lr = new LogReader(fn, &events);
    lrs.append(lr);
  }

  window.resize(250, 150);
  window.setWindowTitle("Simple example");
  window.show();

  return app.exec();
}

