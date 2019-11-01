#ifndef FRAMEREADER_HPP
#define FRAMEREADER_HPP

#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <list>
#include <condition_variable>

#include "channel.hpp"

// independent of QT, needs ffmpeg
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}


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

  bool valid = true;
  char url[0x400];
};

#endif

