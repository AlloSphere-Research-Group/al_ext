#ifndef INCLUDE_AL_VIDEO_PLAYER_HPP
#define INCLUDE_AL_VIDEO_PLAYER_HPP

#include <string>
#include <mutex>
// #include <condition_variable>
#include <atomic>
#include <vlc/vlc.h>
#include "al/graphics/al_Texture.hpp"

namespace al {

class VideoPlayer {
public:
  VideoPlayer();
  ~VideoPlayer();

  bool open(std::string fullpath, double playbackRate = 1.0, bool loop = true);
  bool play();
  bool pause();
  bool stop();
  bool isPlaying();
  
  double duration();
  double position();
  void position(double pos);
  
  double playbackRate();
  void playbackRate(double rate);
  
  double volume();
  void volume(double vol);
  
  bool update();
  Texture& texture(){ return mTexture; };
  
  unsigned int width() const;
  unsigned int height() const;
  const unsigned char *pixels() const;

private:
  static void *lock(void *data, void **p_pixels);
  static void unlock(void *data, void *id, void *const *p_pixels);
  static void display(void *data, void *id);

  libvlc_instance_t *mVlcInstance = nullptr;
  libvlc_media_t *mVlcMedia = nullptr;
  libvlc_media_player_t *mVlcPlayer = nullptr;
  
  std::mutex mMutex;
//   std::condition_variable mFrameReady;
  unsigned char *mPixels = nullptr;
  unsigned char *mBackBuffer = nullptr;
  size_t mBufferSize = 0;
  std::atomic<bool> mNewFrame{false};
  
  unsigned int mWidth = 0;
  unsigned int mHeight = 0;
  Texture mTexture;
};

} // namespace al

#endif