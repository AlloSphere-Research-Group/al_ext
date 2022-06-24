#include "al_ext/video/al_VideoTexture.hpp"

#include <libavutil/hwcontext.h>
#include <libavutil/opt.h>

using namespace al;

void VideoTexture::init() {
  video_state.format_ctx = nullptr;

  video_state.video_st_idx = -1;
  video_state.video_st = nullptr;
  video_state.video_ctx = nullptr;
  video_state.sws_ctx = nullptr;

  video_state.audio_enabled = true;
  video_state.audio_st_idx = -1;
  video_state.audio_st = nullptr;
  video_state.audio_ctx = nullptr;
  video_state.audio_sample_size = 0;
  video_state.audio_channel_size = 0;
  video_state.audio_frame_size = 0;

  video_state.video_clock = 0;
  video_state.audio_clock = 0;
  video_state.master_clock = 0;
  video_state.last_frame_pts = 0;

  video_state.seek_requested = 0;
  video_state.seek_pos = 0;
  video_state.seek_offset = 0;

  video_state.dry_run = false;
  video_state.global_quit = 0;

  video_state.mVideoFrames.resize(VIDEO_BUFFER_SIZE);
  video_state.mVideoFramesRead = 0;
  video_state.mVideoFramesWrite = 0;
  video_state.hw_device_ctx = NULL;

  enum AVHWDeviceType type = AV_HWDEVICE_TYPE_NONE;
  fprintf(stderr, "Available device types:");
  while ((type = av_hwdevice_iterate_types(type)) != AV_HWDEVICE_TYPE_NONE)
    fprintf(stderr, " %s", av_hwdevice_get_type_name(type));
  fprintf(stderr, "\n");
}

bool VideoTexture::updateTexture(double time) {
  bool newFrame = false;
  uint8_t *frame = getVideoFrame(time);
  if (frame) {
    mTexture.submit(frame);
    newFrame = true;
  }
  releaseVideoFrame();
  return newFrame;
}

Texture &VideoTexture::texture() { return mTexture; }

bool VideoTexture::load(const char *url) {
  // open file
  if (avformat_open_input(&video_state.format_ctx, url, NULL, NULL) < 0) {
    std::cerr << "Could not open file: " << url << std::endl;
    return false;
  }

  // retrieve stream information
  if (avformat_find_stream_info(video_state.format_ctx, NULL) < 0) {
    std::cerr << "Could not find stream info: " << url << std::endl;
    return false;
  }

  // print info
  av_dump_format(video_state.format_ctx, 0, url, 0);

  // find video & audio stream
  // TODO: choose stream in case of multiple streams
  for (int i = 0; i < video_state.format_ctx->nb_streams; ++i) {
    if (video_state.format_ctx->streams[i]->codecpar->codec_type ==
            AVMEDIA_TYPE_VIDEO &&
        video_state.video_st_idx < 0) {
      video_state.video_st_idx = i;
      if (video_state.audio_st_idx > 0)
        break;
    }

    if (video_state.format_ctx->streams[i]->codecpar->codec_type ==
            AVMEDIA_TYPE_AUDIO &&
        video_state.audio_st_idx < 0) {
      video_state.audio_st_idx = i;
      if (video_state.video_st_idx > 0)
        break;
    }
  }
  bool useHWDecode = false;

  if (!useHWDecode) {
    if (video_state.video_st_idx == -1) {
      std::cerr << "Could not find video stream" << std::endl;
      return false;
    } else if (!stream_component_open(&video_state, video_state.video_st_idx)) {
      std::cerr << "Could not open video codec" << std::endl;
      return false;
    }

    if (video_state.audio_st_idx == -1) {
      // no audio stream
      // TODO: consider audio only files

      video_state.audio_enabled = false;
    } else if (video_state.audio_enabled) {
      if (!stream_component_open(&video_state, video_state.audio_st_idx)) {
        std::cerr << "Could not open audio codec" << std::endl;
        return false;
      }
    }
    video_state.seek_offset = 5 * fps();
  }

  if (useHWDecode) {
    AVHWDeviceType type = AV_HWDEVICE_TYPE_CUDA;
    int err;
    struct AVCodec *decoder;

    video_state.video_st_idx = av_find_best_stream(
        video_state.format_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder, 0);
    video_state.video_st =
        video_state.format_ctx->streams[video_state.video_st_idx];
    for (int i = 0;; i++) {
      const AVCodecHWConfig *config = avcodec_get_hw_config(decoder, i);
      if (!config) {
        fprintf(stderr, "Decoder %s does not support device type %s.\n",
                decoder->name, av_hwdevice_get_type_name(type));
      }
      if (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
          config->device_type == type) {
        hw_pix_fmt = config->pix_fmt;
        break;
      }
    }

    AVCodecContext *decoder_ctx = NULL;
    if (!(decoder_ctx = avcodec_alloc_context3(decoder)))
      return AVERROR(ENOMEM);
    if (avcodec_parameters_to_context(decoder_ctx,
                                      video_state.video_st->codecpar) < 0)
      return -1;

    decoder_ctx->get_format = get_hw_format;

    if ((err = av_hwdevice_ctx_create(&video_state.hw_device_ctx, type, NULL,
                                      NULL, 0)) < 0) {
      fprintf(stderr, "Failed to create specified HW device.\n");
      return err;
    }
    decoder_ctx->hw_device_ctx = av_buffer_ref(video_state.hw_device_ctx);

    video_state.video_ctx = decoder_ctx;
    if (avcodec_open2(decoder_ctx, decoder, NULL) < 0) {
      std::cerr << "Could not open codec" << std::endl;
      return false;
    }
    std::cout << "initialized HW decoding" << std::endl;
    auto *vs = &video_state;
    //  switch (decoder_ctx->codec_type) {
    //  case AVMEDIA_TYPE_AUDIO: {
    //       video_state.audio_st =
    //       video_state.format_ctx->streams[stream_index];
    //       video_state.audio_ctx = decoder_ctx;

    //      // set parameters
    //      vs->audio_sample_size =
    //      av_get_bytes_per_sample(vs->audio_ctx->sample_fmt); if
    //      (vs->audio_sample_size < 0) {
    //          std::cerr << "Failed to calculate data size" << std::endl;
    //          return false;
    //      }

    //      vs->audio_channel_size = vs->audio_sample_size *
    //      vs->audio_ctx->frame_size;

    //      vs->audio_frame_size = vs->audio_sample_size *
    //      vs->audio_ctx->frame_size *
    //                             vs->audio_ctx->channels;
    //  } break;
    //  case AVMEDIA_TYPE_VIDEO: {
    vs->video_st = vs->format_ctx->streams[0];
    vs->video_ctx = decoder_ctx;

    // initialize SWS context for software scaling
    vs->sws_ctx = sws_getContext(vs->video_ctx->width, vs->video_ctx->height,
                                 vs->video_ctx->pix_fmt, vs->video_ctx->width,
                                 vs->video_ctx->height, AV_PIX_FMT_RGBA,
                                 SWS_FAST_BILINEAR, NULL, NULL, NULL);
    int numBytes = av_image_get_buffer_size(
        AV_PIX_FMT_RGBA, vs->video_ctx->width, vs->video_ctx->height, 32);
    for (auto &frame : vs->mVideoFrames) {
      frame.data.resize(numBytes);
    }
    //  } break;
    //  default: {
    //      break;
    //  }
    //  }
  }

  // TODO: add initialization notice to videoapp
  return true;
}

void VideoTexture::setDryRun(bool dryRun) { video_state.dry_run = dryRun; }

bool VideoTexture::stream_component_open(VideoTextureState *vs,
                                         int stream_index) {
  // check if stream index is valid
  if (stream_index < 0 || stream_index >= vs->format_ctx->nb_streams) {
    std::cerr << "Invalid stream index" << std::endl;
    return false;
  }

  // retrieve codec
  const AVCodec *codec = nullptr;
  codec = avcodec_find_decoder(
      vs->format_ctx->streams[stream_index]->codecpar->codec_id);
  if (!codec) {
    std::cerr << "Unsupported codec" << std::endl;
    return false;
  }

  // retrieve codec context
  AVCodecContext *codecCtx = nullptr;
  codecCtx = avcodec_alloc_context3(codec);
  if (avcodec_parameters_to_context(
          codecCtx, vs->format_ctx->streams[stream_index]->codecpar) != 0) {
    std::cerr << "Could not copy codec context" << std::endl;
    return false;
  }

  { // Threading
    codecCtx->thread_count = 6;

    //    codecCtx->thread_type = FF_THREAD_FRAME;
    //  codecCtx->thread_type = FF_THREAD_SLICE;
  }
  // initialize the AVCodecContext to use the given AVCodec
  if (avcodec_open2(codecCtx, codec, NULL) < 0) {
    std::cerr << "Could not open codec" << std::endl;
    return false;
  }

  switch (codecCtx->codec_type) {
  case AVMEDIA_TYPE_AUDIO: {
    vs->audio_st = vs->format_ctx->streams[stream_index];
    vs->audio_ctx = codecCtx;

    // set parameters
    vs->audio_sample_size = av_get_bytes_per_sample(vs->audio_ctx->sample_fmt);
    if (vs->audio_sample_size < 0) {
      std::cerr << "Failed to calculate data size" << std::endl;
      return false;
    }

    vs->audio_channel_size = vs->audio_sample_size * vs->audio_ctx->frame_size;

    vs->audio_frame_size = vs->audio_sample_size * vs->audio_ctx->frame_size *
                           vs->audio_ctx->channels;
  } break;
  case AVMEDIA_TYPE_VIDEO: {
    vs->video_st = vs->format_ctx->streams[stream_index];
    vs->video_ctx = codecCtx;

    // initialize SWS context for software scaling
    vs->sws_ctx = sws_getContext(vs->video_ctx->width, vs->video_ctx->height,
                                 vs->video_ctx->pix_fmt, vs->video_ctx->width,
                                 vs->video_ctx->height, AV_PIX_FMT_RGBA,
                                 SWS_FAST_BILINEAR, NULL, NULL, NULL);
    int numBytes = av_image_get_buffer_size(
        AV_PIX_FMT_RGBA, vs->video_ctx->width, vs->video_ctx->height, 32);
    for (auto &frame : vs->mVideoFrames) {
      frame.data.resize(numBytes);
    }
  } break;
  default: {
    break;
  }
  }

  video_state.videoFrameSignal.notify_one();

  return true;
}

void VideoTexture::start() {
  // if threads are already running, close them
  if (decode_thread != nullptr) {
    stop();
  }

  if (mUseTexture) {
    // generate texture
    mTexture.filter(Texture::LINEAR);
    mTexture.wrap(Texture::REPEAT, Texture::CLAMP_TO_EDGE,
                  Texture::CLAMP_TO_EDGE);
    mTexture.create2D(width(), height(), Texture::RGBA8, Texture::RGBA,
                      Texture::UBYTE);
  }

  // check if video streams is valid
  if (video_state.video_st != nullptr) {
    video_state.readyMarker = std::make_unique<std::promise<void>>();
    auto future = video_state.readyMarker->get_future();
    decode_thread = new std::thread(decodeThreadFunction, &video_state);
    future.wait();
    video_state.frameSignalLock.lock();
    video_state.videoFrameSignal.notify_one();
    video_state.frameSignalLock.unlock();
    // Fill buffer before returning
    while (readFramesInBuffer() < (VIDEO_BUFFER_SIZE - 1)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  if (!decode_thread) {
    std::cerr << "Could not start decoding thread" << std::endl;
    stop();
  }
}

bool VideoTexture::readNextPacket(VideoTextureState *vs, AVPacket *packet) {
  if (av_read_frame(vs->format_ctx, packet) < 0) {
    if (vs->format_ctx->pb->error != 0) {
      std::cerr << "Error reading frame" << std::endl;
      return false;
    }
  }
  return true;
}

void VideoTexture::decodeThreadFunction(VideoTextureState *vs) {
  // // allocate packet
  AVPacket *packet = av_packet_alloc();
  if (!packet) {
    std::cerr << "Could not allocate packet" << std::endl;
    vs->global_quit = -1;
    return;
  }

  // allocate frame
  AVFrame *frame = av_frame_alloc();
  if (!frame) {
    std::cerr << "Could not allocate frame" << std::endl;
    vs->global_quit = -1;
    return;
  }

  // allocate frame
  AVFrame *frameRGB = av_frame_alloc();
  if (!frameRGB) {
    std::cerr << "Could not allocate frameRGB" << std::endl;
    vs->global_quit = -1;
    return;
  }

  vs->seek_applied = false;
  vs->readyMarker->set_value(); // report thread is initialized
  // check global quit flag
  if (!readNextPacket(vs, packet)) {
    vs->global_quit = 1;
  }
  if (packet->stream_index == vs->video_st_idx) {
    if (avcodec_send_packet(vs->video_ctx, packet) < 0) {
      std::cerr << "Error sending video packet for decoding" << std::endl;
    }
  }
  goto after_wait;
  while (vs->global_quit == 0) {
    {
      std::unique_lock<std::mutex> lk(vs->frameSignalLock);
      vs->videoFrameSignal.wait(lk);
      if (vs->verbose) {
        std::cout << "Wake up to fill buffer" << std::endl;
      }
    }
  after_wait:
    // seeking
    if (vs->seek_requested) {
      if (vs->last_seek != vs->seek_pos) {

        vs->mVideoFramesRead = vs->mVideoFramesWrite = 0;
        int video_stream_index = -1;
        double video_seek_target = vs->seek_pos + vs->seek_offset;

        if (vs->video_st) {
          video_stream_index = vs->video_st_idx;
        }
        video_seek_target *=
            (vs->format_ctx->streams[video_stream_index]->time_base.den /
             (double)vs->format_ctx->streams[video_stream_index]
                 ->time_base.num);
        //      if (audio_stream_index >= 0) {
        //        audio_seek_target = av_rescale_q(
        //            audio_seek_target, av_get_time_base_q(),
        //            vs->format_ctx->streams[audio_stream_index]->time_base);
        //      }
        //      auto m_streamTimebase =
        //          av_q2d(vs->video_st->time_base) * 1000.0 * 10000.0;
        //      double fps =
        //          av_q2d(av_guess_frame_rate(vs->format_ctx, vs->video_st,
        //          NULL));
        //      video_seek_target = video_seek_target * fps;

        if (vs->verbose) {
          std::cout << "[[ " +
                           std::to_string(av_get_time_base_q().num /
                                          (double)av_get_time_base_q().den) +
                           "]]"
                    << std::endl;
          std::cout << "[seek to " + std::to_string(vs->seek_pos) + "  " +
                           std::to_string(video_seek_target) + "]"
                    << std::endl;
        }
        // TODO Should check if we have the frame in the buffer
        vs->seek_start_pos = vs->video_clock;
        vs->last_seek = vs->seek_pos;
        int flags = 0; // AVSEEK_FLAG_BACKWARD | AVSEEK_FLAG_ANY
        if (vs->seek_pos < vs->video_clock) {
          flags |= AVSEEK_FLAG_BACKWARD;
        }
        int ret = av_seek_frame(vs->format_ctx, video_stream_index,
                                video_seek_target, flags);
        if (ret < 0) {
          std::cerr << "Error seeking" << std::endl;
        }
        avcodec_flush_buffers(vs->video_ctx);
        //      avcodec_flush_buffers(vs->audio_ctx);

        //      if (vs->audio_st && vs->audio_enabled)
        //        ret &= av_seek_frame(vs->format_ctx, audio_stream_index,
        //                             audio_seek_target, vs->seek_flags);
        //      if (!readNextPacket(vs, packet)) {
        //        vs->global_quit = 1;
        //      }
        vs->seek_applied = true;
      }
      vs->seek_requested = false;
      // std::cout << "seek end" << std::endl;
    }

    while (vs->global_quit == 0) {

      // check the type of packet
      if (packet->stream_index == vs->video_st_idx) {

        auto newIndex = vs->mVideoFramesWrite + 1;
        if (newIndex == vs->mVideoFrames.size()) {
          newIndex = 0;
        }
        if (newIndex == vs->mVideoFramesRead) {
          break;
        }
        int ret = avcodec_receive_frame(vs->video_ctx, frame);
        //          frame->best_effort_timestamp == AV_NOPTS_VALUE
        //              ? frame->pts
        //              : frame->best_effort_timestamp;
        if (ret == AVERROR(EAGAIN)) {
          if (!readNextPacket(vs, packet)) {
            vs->global_quit = 1;
          } else {
            // send raw compressed video data in AVPacket to decoder
            if (packet->stream_index == vs->video_st_idx) {
              if (avcodec_send_packet(vs->video_ctx, packet) < 0) {
                std::cerr << "Error sending video packet for decoding"
                          << std::endl;
              }
            }
          }
          goto after_wait; // Need more packets for frame
        } else if (ret != 0) {
          char errbuf[128];
          av_strerror(ret, errbuf, 128);
          std::cout << "ERROR: " << errbuf << " --  " << AVERROR(EAGAIN)
                    << std::endl;
        }
        // get the estimated time stamp
        double pts = frame->best_effort_timestamp;

        // if guess failed
        if (pts == AV_NOPTS_VALUE) {
          // if we don't have a pts, use the video clock
          pts = vs->video_clock;
        } else {
          // convert pts using video stream's time base
          pts *= av_q2d(vs->video_st->time_base);

          // if we have pts, set the video_clock to it
          vs->video_clock = pts;
        }

        // update video clock if frame is delayed
        vs->video_clock +=
            0.5 * av_q2d(vs->video_st->time_base) * frame->repeat_pict;

        if (vs->seek_applied) {
          if (frame->best_effort_timestamp == AV_NOPTS_VALUE) {
            std::cerr << "Iframe - no time stamp" << std::endl;
            break;
          } else {
            auto currentPts = frame->best_effort_timestamp *
                              (vs->video_st->time_base.num /
                               (double)vs->video_st->time_base.den);
            if (currentPts - 0.0001 > vs->seek_start_pos &&
                currentPts - 0.0001 > (vs->seek_pos + vs->seek_offset)) {
              if (vs->verbose) {
                std::cout << "Reseek back" << std::endl;
              }
              vs->last_seek = currentPts;
              vs->seek_requested = 1;

              goto after_wait; // Need more packets for frame
            }
            if (currentPts + 0.0001 < (vs->seek_pos + vs->seek_offset)) {
              goto after_wait;
            }
          }
        }
        // check if entire frame was decoded
        if (ret == AVERROR(EAGAIN)) {
          // need more data
          break;
        } else if (ret == AVERROR_EOF) {
          vs->global_quit = 1;
          break;
        } else if (ret < 0) {
          std::cerr << "Error while decoding" << std::endl;
          vs->global_quit = -1;
          break;
        }

        // if (vs->video_clock < vs->master_clock) {
        //   break;
        // }
        // Setup pointers and linesize for dst frame and image data buffer

        if (!vs->dry_run) {
          av_image_fill_arrays(
              frameRGB->data, frameRGB->linesize,
              vs->mVideoFrames[vs->mVideoFramesWrite].data.data(),
              AV_PIX_FMT_RGBA, vs->video_ctx->width, vs->video_ctx->height, 32);
          // scale image in frame and put results in frameRGB
          sws_scale(vs->sws_ctx, (uint8_t const *const *)frame->data,
                    frame->linesize, 0, vs->video_ctx->height, frameRGB->data,
                    frameRGB->linesize);
        }
        vs->mVideoFrames[vs->mVideoFramesWrite].consumed = false;
        vs->mVideoFrames[vs->mVideoFramesWrite].pts = pts;
        vs->mVideoFrames[vs->mVideoFramesWrite].pictureType = frame->pict_type;
        vs->seek_applied = false; // Found I-frame after seek.

        if (vs->verbose) {
          std::cout << "Wrote frame in buffer: " << pts << std::endl;
        }
        vs->mVideoFramesWrite = newIndex;
      } else if (packet->stream_index == vs->audio_st_idx) {
        // skip if audio has been disabled
        if (!vs->audio_enabled) {
          av_packet_unref(packet);
          goto after_wait;
        }
      }

      // wipe the packet
    }
  }
  // free the memory
  //  av_freep(&audio_out);
  av_frame_free(&frameRGB);
  av_frame_free(&frame);
  av_packet_free(&packet);
}

uint8_t *VideoTexture::getVideoFrame(double requestedTime, double *frameTime) {

  if (video_state.mVideoFramesRead != video_state.mVideoFramesWrite) {
    // Frames are available in buffer
    auto *nextFrame = &video_state.mVideoFrames[video_state.mVideoFramesRead];
    if (requestedTime == -1) {
      // Just deliver next frame
      nextFrame->consumed = true;
      return nextFrame->data.data();
    } else {
      double frameInterval = 1.0 / fps();
      int frameLookAhead = 0;
      int maxDriftFrames = 3;

      // Perfect match
      if (requestedTime >= (nextFrame->pts - 0.0001) &&
          (requestedTime - nextFrame->pts + 0.0001) < frameInterval) {
        nextFrame->consumed = true;
        if (frameTime) {
          *frameTime = nextFrame->pts;
        }
        return nextFrame->data.data();
      }
      // If time is a little behind the buffer, just return frame in buffer, but
      // dont consume it. Time should catch up to this frame.
      if (nextFrame->pts > requestedTime) {
        if ((nextFrame->pts - requestedTime) <
            (frameInterval * maxDriftFrames)) {
          // return frame without consuming it
          if (frameTime) {
            *frameTime = nextFrame->pts;
          }
          return nextFrame->data.data();
        }
      }

      if (requestedTime > (nextFrame->pts - 0.0001) &&
          requestedTime <=
              (nextFrame->pts + ((VIDEO_BUFFER_SIZE - 1) * frameInterval))) {
        // Check to see if requested time is in buffer (in case time has moved
        // faster and we need to skip frames)
        while (nextFrame && nextFrame->pts - 0.0001 < requestedTime &&
               nextFrame->pts != requestedTime) {
          if (video_state.mVideoFramesRead == video_state.mVideoFramesWrite) {
            nextFrame = nullptr;
            break;
          }
          nextFrame->consumed = true;
          nextFrame = &video_state.mVideoFrames[video_state.mVideoFramesRead];
          auto val = video_state.mVideoFramesRead;
          if (val + 1 == video_state.mVideoFrames.size()) {
            video_state.mVideoFramesRead = 0;
          } else {
            video_state.mVideoFramesRead = val + 1;
          }
        }
      }

      if (nextFrame && nextFrame->pts >= requestedTime &&
          (nextFrame->pts - requestedTime) < frameInterval) {
        // Frame available in buffer
        nextFrame->consumed = true;
        if (frameTime) {
          *frameTime = nextFrame->pts;
        }
        return nextFrame->data.data();
      }

      {
        // If we get here, we need to seek
        auto pos = requestedTime + (frameInterval * frameLookAhead);
        seek(pos);
      }
      video_state.videoFrameSignal.notify_one();
    }
  } else {
    double frameInterval = 1.0 / fps();
    // No frames available in buffer, seek and fill buffer
    if (requestedTime >= 0 &&
        (requestedTime<(video_state.video_clock - 0.0001) |
                       fabs(requestedTime - video_state.video_clock)>
             frameInterval *
         5)) {
      seek(requestedTime);
    }
    video_state.videoFrameSignal.notify_one();
  }

  return nullptr;
}

double VideoTexture::getCurrentFrameTime() {

  if (video_state.mVideoFramesRead != video_state.mVideoFramesWrite) {
    auto *nextFrame = &video_state.mVideoFrames[video_state.mVideoFramesRead];
    return nextFrame->pts;
  }
  return -1;
}

int VideoTexture::getCurrentFrameType() {

  if (video_state.mVideoFramesRead != video_state.mVideoFramesWrite) {
    auto *nextFrame = &video_state.mVideoFrames[video_state.mVideoFramesRead];
    return nextFrame->pictureType;
  }
  return AV_PICTURE_TYPE_NONE;
}

void VideoTexture::releaseVideoFrame() {
  while (video_state.mVideoFramesRead != video_state.mVideoFramesWrite) {
    if (video_state.mVideoFrames[video_state.mVideoFramesRead].consumed) {
      int val = video_state.mVideoFramesRead;
      if (mVerbose) {
        std::cout << " releasing buffer: " << val << " @ "
                  << video_state.mVideoFrames[video_state.mVideoFramesRead].pts
                  << std::endl;
      }
      if (val + 1 == video_state.mVideoFrames.size()) {
        video_state.mVideoFramesRead = 0;
      } else {
        video_state.mVideoFramesRead = val + 1;
      }
    } else {
      video_state.videoFrameSignal.notify_one();
      return;
    }
  }
  video_state.videoFrameSignal.notify_one();
}

// uint8_t *VideoDecoder2::getAudioFrame(double external_clock) {
//  if (video_state.seek_requested) {
//    return nullptr;
//  }

//  // get next audio frame
//  audio_output = audio_buffer.get();
//  if (!audio_output) {
//    return nullptr;
//  }

//  // TODO: implement audio sync to video/external
//  if (video_state.master_sync == MasterSync::AV_SYNC_AUDIO) {
//    // update master clock if audio sync
//    video_state.master_clock = audio_output->pts;
//  }
//  // else {
//  //   // difference between target pts and current master clock
//  //   double audio_diff = audio_output.pts - video_state.master_clock;

//  //   // sync audio if needed
//  //   if (fabs(audio_diff) > AV_SYNC_THRESHOLD) {
//  //     std::cout << " audio_diff: " << audio_diff << std::endl;
//  //   }
//  // }

//  return audio_output->data.data();
//}

void VideoTexture::seek(double time) {
  if (!video_state.seek_requested && !video_state.seek_applied) {
    video_state.seek_pos = time;
    // TODO: check which flag to use
    //    video_state.seek_flags = AVSEEK_FLAG_BACKWARD;
    // video_state.seek_flags = AVSEEK_FLAG_ANY;
    video_state.seek_requested = 1;
    video_state.videoFrameSignal.notify_one();
  } else {
    std::cout << "Seek in progress. Seek ignored: " << time << std::endl;
  }
}

void VideoTexture::setSeekOffset(double time) {
  video_state.seek_offset = time;
}

unsigned int VideoTexture::audioSampleRate() {
  if (video_state.audio_ctx)
    return video_state.audio_ctx->sample_rate;
  return 0;
}

unsigned int VideoTexture::audioNumChannels() {
  if (video_state.audio_ctx)
    return video_state.audio_ctx->channels;
  return 0;
}

unsigned int VideoTexture::audioSamplesPerChannel() {
  if (video_state.audio_ctx)
    return video_state.audio_ctx->frame_size;
  return 0;
}

int VideoTexture::width() {
  if (video_state.video_ctx)
    return video_state.video_ctx->width;
  return 0;
}

int VideoTexture::height() {
  if (video_state.video_ctx)
    return video_state.video_ctx->height;
  return 0;
}

double VideoTexture::fps() {
  double guess = av_q2d(
      av_guess_frame_rate(video_state.format_ctx, video_state.video_st, NULL));
  if (guess == 0) {
    std::cerr << "Could not guess frame rate" << std::endl;
    guess = av_q2d(video_state.format_ctx->streams[video_state.video_st_idx]
                       ->r_frame_rate);
  }
  return guess;
}

int VideoTexture::readFramesInBuffer() {
  auto dist = video_state.mVideoFramesWrite - video_state.mVideoFramesRead;
  if (dist < 0) {
    dist += video_state.mVideoFrames.size();
  }
  return dist;
}

double VideoTexture::getCurrentDecoderTime() { return video_state.video_clock; }

void VideoTexture::cleanup() {
  // Close the audio codec
  avcodec_free_context(&video_state.audio_ctx);
  // Close the video codec
  avcodec_free_context(&video_state.video_ctx);
  // free the sws context
  sws_freeContext(video_state.sws_ctx);
  // Close the video file
  avformat_close_input(&video_state.format_ctx);
}

void VideoTexture::stop() {
  video_state.global_quit = 1;
  video_state.videoFrameSignal.notify_one();

  if (decode_thread) {
    decode_thread->join();
  }
  std::thread *dth = decode_thread;
  decode_thread = nullptr;
  delete dth;

  cleanup();
}
