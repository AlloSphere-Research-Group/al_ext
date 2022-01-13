#include "al/app/al_App.hpp"
#include "al/graphics/al_Shapes.hpp"

#include "al_ext/video/al_VideoDecoder.hpp"

class VideoApp : public App {
public:
  virtual void onCreate() override {

    videoDecoder.enableAudio(false);
    if (!videoDecoder.load(mVideoFileToLoad.c_str())) {
      std::cerr << "Error loading video file" << std::endl;
      //            quit();
    }
    // generate texture
    tex.filter(Texture::LINEAR);
    tex.wrap(Texture::REPEAT, Texture::CLAMP_TO_EDGE, Texture::CLAMP_TO_EDGE);
    tex.create2D(videoDecoder.width(), videoDecoder.height(), Texture::RGBA8,
                 Texture::RGBA, Texture::UBYTE);

    fps(videoDecoder.fps());
    mPlaying = true;

    aspectRatio = videoDecoder.width() / (double)videoDecoder.height();
    videoDecoder.start();
  }

  virtual void onAnimate(al_sec dt) override {

    bool timeChanged = false;
    if (mPlaying) {
      wallTime += dt;
      timeChanged = true;
    }
    if (wallTime < previousTime) {
      videoDecoder.stream_seek((int64_t)(wallTime * AV_TIME_BASE), -10);
      timeChanged = true;
    } else if (wallTime - previousTime > 3.0 / 30.0) {

      videoDecoder.stream_seek((int64_t)(wallTime * AV_TIME_BASE), 10);
      timeChanged = true;
    } else if (wallTime != previousTime) {
      videoDecoder.stream_seek((int64_t)(wallTime * AV_TIME_BASE), 10);
      timeChanged = true;
    }
    if (timeChanged) {
      uint8_t *frame = videoDecoder.getVideoFrame(wallTime);

      if (frame) {
        tex.submit(frame);
      }
    }
    previousTime = wallTime;
  }

  virtual void onDraw(Graphics &g) override {
    g.clear(0.1f);
    g.pushMatrix();
    g.translate(0, 0, -3);
    g.rotate(180, 0, 1, 0);
    g.quad(tex, -0.5 * aspectRatio, -0.5, aspectRatio, 1, true);
    g.popMatrix();
  }

  void setVideoFile(std::string videoFileUrl) {
    mVideoFileToLoad = videoFileUrl;
  }

  double wallTime{0.0};
  double previousTime{0.0};
  double aspectRatio{1.0};

private:
  Texture tex;

  VideoDecoder videoDecoder;
  std::string mVideoFileToLoad;

  bool mPlaying{false};
};

int main() {
  VideoApp app;
  // Set vide file here
  app.setVideoFile("C:/Users/Andres/Downloads/Lw Kt Edit 0103 Good 75Mbps 8K "
                   "360-4k-30fps-noaudio.m4v");

  app.start();
  return 0;
}
