#include "FrameReader.hpp"
#include <assert.h>

FrameReader::FrameReader(const char *fn) {
  int ret;
  char url[0x400];
  snprintf(url, sizeof(url)-1, "http://data.comma.life/%s", fn);

  avformat_network_init();
  av_register_all();
  if (avformat_open_input(&pFormatCtx, url, NULL, NULL) != 0) {
    fprintf(stderr, "error loading %s\n", url);
  }
  av_dump_format(pFormatCtx, 0, url, 0);

  auto pCodecCtxOrig = pFormatCtx->streams[0]->codec;
  auto pCodec = avcodec_find_decoder(pCodecCtxOrig->codec_id);
  assert(pCodec != NULL);

  pCodecCtx = avcodec_alloc_context3(pCodec);
  ret = avcodec_copy_context(pCodecCtx, pCodecCtxOrig);
  assert(ret == 0);

  ret = avcodec_open2(pCodecCtx, pCodec, NULL);
  assert(ret >= 0);

	sws_ctx = sws_getContext(width, height, PIX_FMT_YUV420P,
													 width, height, PIX_FMT_BGR24,
													 SWS_BILINEAR, NULL, NULL, NULL);

  t = new std::thread([&](){
    AVPacket *pkt = (AVPacket *)malloc(sizeof(AVPacket));
    while (av_read_frame(pFormatCtx, pkt)>=0) {
      //printf("%d pkt %d %d\n", pkts.size(), pkt->size, pkt->pos);
      pkts.push_back(pkt);
      pkt = (AVPacket *)malloc(sizeof(AVPacket));
    }
    free(pkt);
    printf("framereader download done\n");
  });


  /*    int frameFinished;
      AVFrame *pFrame=av_frame_alloc();
      avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet);
      if (!frameFinished) {
        printf("NO FRAME\n");
      }*/

  /*printf("started decode thread\n");
  t = new std::thread([&](){
    printf("hello from decode thread\n");
    AVPacket packet;
    int i = 0;
    while (av_read_frame(pFormatCtx, &packet)>=0) {
      int frameFinished;
      AVFrame *pFrame=av_frame_alloc();
      avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet);
      if (frameFinished) {
        printf("got frame: %d\n", i);
        frames_mutex.lock();
        AVFrame *pFrameRGB = toRGB(pFrame);
        frames.push_back((uint8_t*)pFrameRGB->data[0]);
        frames_mutex.unlock();
        i++;
        //if (i==10) break;
      }
    }
  });*/
}

AVFrame *FrameReader::toRGB(AVFrame *pFrame) {
  AVFrame *pFrameRGB = av_frame_alloc();
  int numBytes = avpicture_get_size(PIX_FMT_BGR24, pFrame->width, pFrame->height);
  uint8_t *buffer = (uint8_t *)av_malloc(numBytes*sizeof(uint8_t));
  avpicture_fill((AVPicture *)pFrameRGB, buffer, PIX_FMT_BGR24, pFrame->width, pFrame->height);
	sws_scale(sws_ctx, (uint8_t const * const *)pFrame->data,
						pFrame->linesize, 0, pFrame->height,
						pFrameRGB->data, pFrameRGB->linesize);
  return pFrameRGB;
}

uint8_t *FrameReader::get(int idx) {
  waitForReady();
  // TODO: one line?
  auto it = cache.find(idx);
  if (it != cache.end()) {
    return it->second;
  }

  AVFrame *pFrame;
  uint8_t *dat;
  int gop = idx - idx%15;
  //for (int i = idx - idx%15; i <= idx; i++) {
  for (int i = gop; i <= gop+15; i++) {
    if (i >= pkts.size()) break;
    printf("decode %d\n", i);
    int frameFinished;
    pFrame = av_frame_alloc();
    avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, pkts[i]);
    dat = toRGB(pFrame)->data[0];
    cache.insert(std::make_pair(i, dat));
  }
  return dat;
}

