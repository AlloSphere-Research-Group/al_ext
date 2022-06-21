#ifndef AL_VIDEODECODER_HPP
#define AL_VIDEODECODER_HPP

#ifdef AL_EXT_LIBAV
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <atomic>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

#include "al/graphics/al_Texture.hpp"
#include "al/system/al_Time.hpp"

using namespace al;

static enum AVPixelFormat hw_pix_fmt;
// static const int AUDIO_BUFFER_SIZE = 8;
// static const double AV_SYNC_THRESHOLD = 1.5 / 30.0;
// static const double AV_NOSYNC_THRESHOLD = 3.0;

struct VideoTextureFrame {
  std::vector<uint8_t> data;
  double pts;
  AVPictureType pictureType = AV_PICTURE_TYPE_NONE;
  bool consumed; // Should release frame (consumed at correct timestamp)
  bool seeking;
};

struct VideoTextureState {
  // ** File I/O Context **
  AVFormatContext *format_ctx;

  // ** Video Stream **
  int video_st_idx;
  AVStream *video_st;
  AVCodecContext *video_ctx;
  struct SwsContext *sws_ctx;

  // ** Audio Stream **
  bool audio_enabled = false;
  int audio_st_idx;
  AVStream *audio_st;
  AVCodecContext *audio_ctx;
  int audio_sample_size;
  int audio_channel_size;
  int audio_frame_size;

  // ** Sync **
  double video_clock;
  double audio_clock;
  double master_clock;
  double last_frame_pts;

  // ** Seek **
  bool seek_requested;
  //  int seek_flags;
  double seek_start_pos;
  double seek_pos;

  std::vector<VideoTextureFrame> mVideoFrames;
  int32_t mVideoFramesRead = 0;
  int32_t mVideoFramesWrite = 0;
  std::condition_variable videoFrameSignal;
  std::mutex frameSignalLock;

  //
  std::unique_ptr<std::promise<void>> readyMarker;

  bool dry_run;
  // ** Global Quit Flag **
  int global_quit;
  bool verbose{false};
};

class VideoTexture {
  static const int VIDEO_BUFFER_SIZE = 8;

public:
  VideoTexture(bool useTexture = true) {
    mUseTexture = useTexture;
    init();
  }
  ~VideoTexture() { stop(); }

  // load video file
  bool load(const char *url);
  // initialize video state
  void init();
  // start the threads
  void start();

  // Call updateTexture() or getVideoFrame() not both.
  //  updateTexture() calls getVideoFrame() and releaseVideoFrame() internally.
  bool updateTexture(double time); // true on new texture
  Texture &texture();

  // get the next video/audio frame
  uint8_t *getVideoFrame(double time = -1);
  // Must be called once after getVideoFrame()
  void releaseVideoFrame();

  double getCurrentFrameTime(); // -1.0 means no frame available in buffer
  double getCurrentDecoderTime();
  int getCurrentFrameType();
  int readFramesInBuffer(); // Number of frames available in buffer

  // seek position in video file
  void seek(double time);

  // free memories
  void cleanup();

  // stop the video reader
  void stop();

  // get video parameters
  int width();
  int height();
  double fps();

  // file has audio stream
  bool hasAudio() { return video_state.audio_st != nullptr; }

  // get audio parameters
  unsigned int audioSampleRate();
  unsigned int audioNumChannels();
  unsigned int audioSamplesPerChannel();

  // Skip copying pixels when reading through file
  void setDryRun(bool dryRun);

  void setVerbose(bool verbose) { video_state.verbose = verbose; }

private:
  // open & initialize video/audio stream components
  bool stream_component_open(VideoTextureState *vs, int stream_index);

  // thread functions for decoding and video
  static void decodeThreadFunction(VideoTextureState *vs);
  static bool readNextPacket(VideoTextureState *vs, AVPacket *packet);

  // // attempt to guess proper timestamps for decoded video frames
  // int64_t guess_correct_pts(AVCodecContext *ctx, int64_t &reordered_pts,
  //                           int64_t &dts);

  // // update the pts
  // double synchronize_video(AVFrame *src_frame, double &pts);

  // // inserts decoded frame into picture queue
  // bool queue_picture(AVFrame *qFrame, double &pts);

  // // decode audio frame
  // // returns size of decoded audio data if successful, negative on fail
  // int audio_decode_frame();

  // // get the current audio reference clock
  // double get_audio_clock();

  VideoTextureState video_state;

  //  MediaFrame *video_output;
  //  MediaBuffer video_buffer;

  //  MediaFrame *audio_output;
  //  MediaBuffer audio_buffer;

  //  std::atomic<bool> delay_next_frame{false};
  //  std::atomic<bool> skip_next_frame{false};

  // ** Threads **
  std::thread *decode_thread{nullptr};

  Texture mTexture;
  bool mUseTexture;

  bool mVerbose{false};

  // HW acceleration
  AVBufferRef *hw_device_ctx;
  int hw_decoder_init(AVCodecContext *ctx, const enum AVHWDeviceType type) {
    int err = 0;

    if ((err = av_hwdevice_ctx_create(&hw_device_ctx, type, NULL, NULL, 0)) <
        0) {
      fprintf(stderr, "Failed to create specified HW device.\n");
      return err;
    }
    ctx->hw_device_ctx = av_buffer_ref(hw_device_ctx);

    return err;
  }

  static enum AVPixelFormat get_hw_format(AVCodecContext *ctx,
                                          const enum AVPixelFormat *pix_fmts) {
    const enum AVPixelFormat *p;

    for (p = pix_fmts; *p != -1; p++) {
      if (*p == hw_pix_fmt)
        return *p;
    }

    fprintf(stderr, "Failed to get HW surface format.\n");
    return AV_PIX_FMT_NONE;
  }
};

#else
#pragma message("al_ext video extension not built. Do not include this header")
#endif

#endif // AL_VIDEODECODER_HPP
