#include "al_ext/video/al_VideoPlayer.hpp"
#include "al_ext/video/al_VLCInstance.hpp"
#include <iostream>

namespace al {

VideoPlayer::VideoPlayer() {
//   mVlcInstance = libvlc_new(0, nullptr);
//   if (!mVlcInstance) {
//     std::cerr << "Failed to create VLC instance" << std::endl;
//   }
}

VideoPlayer::~VideoPlayer() {
  stop();
  
  delete[] mPixels;
  delete[] mBackBuffer;
  mPixels = nullptr;
  mBackBuffer = nullptr;

  if (mVlcPlayer) {
    libvlc_media_player_release(mVlcPlayer);
    mVlcPlayer = nullptr;
  }
  
  if (mVlcMedia) {
    libvlc_media_release(mVlcMedia);
    mVlcMedia = nullptr;
  }
 
}

bool VideoPlayer::open(std::string fullpath, double playbackRate, bool loop) {

  try {
    libvlc_instance_t* vlc = VLCInstance::get().instance();
  
    // Create media
    mVlcMedia = libvlc_media_new_path(vlc, fullpath.c_str());
    if (!mVlcMedia) {
        std::cerr << "Failed to create media from path: " << fullpath << std::endl;
        return false;
    }

    // Set media options
    if (loop) {
        libvlc_media_add_option(mVlcMedia, "input-repeat=65535");
    }

    // Set up the media player
    mVlcPlayer = libvlc_media_player_new_from_media(mVlcMedia);
    if (!mVlcPlayer) {
        std::cerr << "Failed to create media player" << std::endl;
        return false;
    }

    // Get video dimensions
    libvlc_video_get_size(mVlcPlayer, 0, &mWidth, &mHeight);
    if (mWidth == 0 || mHeight == 0) {
        // If dimensions are not immediately available, we need to wait
        libvlc_media_player_play(mVlcPlayer);
        while (mWidth == 0 || mHeight == 0) {
        libvlc_video_get_size(mVlcPlayer, 0, &mWidth, &mHeight);
        }
        libvlc_media_player_stop(mVlcPlayer);
    }

    // Calculate buffer size based on RGBA format
    mBufferSize = mWidth * mHeight * 4;
    
    // Allocate double buffers
    mPixels = new unsigned char[mBufferSize];
    mBackBuffer = new unsigned char[mBufferSize];
    
    // Clear buffers
    std::memset(mPixels, 0, mBufferSize);
    std::memset(mBackBuffer, 0, mBufferSize);

    mTexture.create2D(mWidth, mHeight, Texture::RGBA8, Texture::RGBA, Texture::UBYTE);

    // Set video callbacks
    libvlc_video_set_callbacks(mVlcPlayer, lock, unlock, display, this);
    
    // Set output format (RGBA)
    libvlc_video_set_format(mVlcPlayer, "RGBA", mWidth, mHeight, mWidth * 4);

    // Set playback rate
    libvlc_media_player_set_rate(mVlcPlayer, playbackRate);
  } catch (const std::exception& e) {
    std::cerr << "Error openning video: " << e.what() << std::endl;
    return false;
  }

  return true;
}

bool VideoPlayer::play() {
  if (!mVlcPlayer) return false;
  return libvlc_media_player_play(mVlcPlayer) == 0;
}

bool VideoPlayer::pause() {
  if (!mVlcPlayer) return false;
  libvlc_media_player_pause(mVlcPlayer);
  return true;
}

bool VideoPlayer::stop() {
  if (!mVlcPlayer) return false;
  libvlc_media_player_stop(mVlcPlayer);
  return true;
}

bool VideoPlayer::isPlaying() {
  if (!mVlcPlayer) return false;
  return libvlc_media_player_is_playing(mVlcPlayer) == 1;
}

double VideoPlayer::duration() {
  if (!mVlcMedia) return 0.0;
  return libvlc_media_get_duration(mVlcMedia) / 1000.0;
}

double VideoPlayer::position() {
  if (!mVlcPlayer) return 0.0;
  return libvlc_media_player_get_position(mVlcPlayer);
}

void VideoPlayer::position(double pos) {
  if (!mVlcPlayer) return;
  libvlc_media_player_set_position(mVlcPlayer, pos);
}

double VideoPlayer::playbackRate() {
  if (!mVlcPlayer) return 1.0;
  return libvlc_media_player_get_rate(mVlcPlayer);
}

void VideoPlayer::playbackRate(double rate) {
  if (!mVlcPlayer) return;
  libvlc_media_player_set_rate(mVlcPlayer, rate);
}

void VideoPlayer::volume(double vol) {
  if (!mVlcPlayer) return;
  libvlc_audio_set_volume(mVlcPlayer, static_cast<int>(vol * 100));
}

double VideoPlayer::volume() {
  if (!mVlcPlayer) return 0.0;
  return libvlc_audio_get_volume(mVlcPlayer) / 100.0;
}

bool VideoPlayer::update() {
  std::unique_lock<std::mutex> lock(mMutex);
  if (mNewFrame.load(std::memory_order_acquire)) {
    // if (mTexture) {
      mTexture.submit(mPixels, GL_RGBA, GL_UNSIGNED_BYTE);
    // }
    mNewFrame.store(false, std::memory_order_release);
    return true;
  }
  return false;
}

void *VideoPlayer::lock(void *data, void **p_pixels) {
  VideoPlayer *player = static_cast<VideoPlayer *>(data);
  std::lock_guard<std::mutex> lock(player->mMutex);
  *p_pixels = player->mBackBuffer; // Write to back buffer
  return nullptr;
}

void VideoPlayer::unlock(void *data, void *id, void *const *p_pixels) {
  VideoPlayer *player = static_cast<VideoPlayer *>(data);
  std::lock_guard<std::mutex> lock(player->mMutex);
  // Swap buffers
  std::swap(player->mPixels, player->mBackBuffer);
  player->mNewFrame.store(true, std::memory_order_release);
  // player->mFrameReady.notify_one();
}

void VideoPlayer::display(void *data, void *id) {
  // Not used in this implementation
}

// void VideoPlayer::setTexture(Texture *tex) {
//   mTexture = tex;
// }

unsigned int VideoPlayer::width() const {
  return mWidth;
}

unsigned int VideoPlayer::height() const {
  return mHeight;
}

const unsigned char *VideoPlayer::pixels() const {
  return mPixels;
}

} // namespace al