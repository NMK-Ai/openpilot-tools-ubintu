#ifndef FRAMEREADER_HPP
#define FRAMEREADER_HPP

#include <vector>
#include <thread>
#include <mutex>

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
  int getRGBSize() { return width*height*3; }
private:
  AVFormatContext *pFormatCtx = NULL;
  AVCodecContext *pCodecCtx = NULL;

	struct SwsContext *sws_ctx = NULL;

	int width=1164;
	int height=874;

  std::vector<uint8_t *> frames;
  std::mutex frames_mutex;

  std::thread *t;
};

#endif

