#ifndef FRAMEREADER_HPP
#define FRAMEREADER_HPP

#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <list>
#include <condition_variable>

// independent of QT, needs ffmpeg
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

template<class item>
class channel {
private:
  std::list<item> queue;
  std::mutex m;
  std::condition_variable cv;
public:
  void put(const item &i) {
    std::unique_lock<std::mutex> lock(m);
    queue.push_back(i);
    cv.notify_one();
  }
  void put_front(const item &i) {
    std::unique_lock<std::mutex> lock(m);
    queue.push_front(i);
    cv.notify_one();
  }
  item get() {
    std::unique_lock<std::mutex> lock(m);
    cv.wait(lock, [&](){ return !queue.empty(); });
    item result = queue.front();
    queue.pop_front();
    return result;
  }
};

class FrameReader {
public:
  FrameReader(const char *fn);
  uint8_t *get(int idx);
  AVFrame *toRGB(AVFrame *);
  void waitForReady() {
    if (!joined) t->join();
    joined = true;
  }
  int getRGBSize() { return width*height*3; }
private:
  AVFormatContext *pFormatCtx = NULL;
  AVCodecContext *pCodecCtx = NULL;

	struct SwsContext *sws_ctx = NULL;

	int width = 1164;
	int height = 874;

  std::vector<AVPacket *> pkts;

  std::thread *t;
  bool joined = false;

  std::thread *t2;

  std::map<int, uint8_t *> cache;
  std::mutex mcache;

  void GOPCache(int idx);
  channel<int> to_cache;
};

#endif

