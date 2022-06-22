#include "al/app/al_App.hpp"
#include "al/graphics/al_Shapes.hpp"

#include "al/graphics/al_Font.hpp"
#include "al_ext/video/al_VideoTexture.hpp"

class VideoApp : public App {
public:
  double time = 0;

  virtual void onInit() override {
    // FPS can't be set on onCreate(), has to be set here
    fps(videoDecoder.fps());
    aspectRatio = videoDecoder.width() / (double)videoDecoder.height();
  }
  virtual void onCreate() override {
    videoDecoder.start(); // Must be run on onCreate()
  }

  virtual void onAnimate(al_sec dt) override {
    time += dt;

    if (mPlaying) {
      if (!videoDecoder.updateTexture(-1)) {
        std::cout << " No new frame" << std::endl;
      }
    }
  }

  virtual void onDraw(Graphics &g) override {
    g.clear(0.1f);
    {
      g.pushMatrix();
      g.translate(0, 0, -3);
      // The video texture will be inverted.
      g.rotate(180, 0, 1, 0);
      g.texture();
      // Get video texture
      auto &tex = videoDecoder.texture();
      // draw it
      g.quad(tex, -0.5 * aspectRatio, -0.5, aspectRatio, 1, true);
      g.popMatrix();
    }
    std::string text =
        "Buffer: " + std::to_string(videoDecoder.readFramesInBuffer()) +
        "/7   " + std::to_string(videoDecoder.getCurrentFrameTime()) + "  " +
        std::to_string(time) +
        " fps: " + std::to_string(graphicsDomain()->fps());
    FontRenderer::render(g, text.c_str(), Vec3d(-0.33, 0.2, -1), 0.03, 24);
  }

  bool loadVideoFile(std::string videoFileUrl) {
    if (!videoDecoder.load(videoFileUrl.c_str())) {
      std::cerr << "Error loading video file" << std::endl;
      return false;
    }
    return true;
  }

  bool onKeyDown(Keyboard const &k) override {
    if (k.key() == ' ') {
      mPlaying = !mPlaying;
    }
    if (k.key() == '.') { // move one frame forward
      if (!videoDecoder.updateTexture(-1)) {
        std::cout << " No new frame" << std::endl;
      } else {
        std::cout << "New frame" << std::endl;
      }
    }
    return true;
  }

  double aspectRatio{1.0};
  bool mPlaying{false};

private:
  Texture tex;

  VideoTexture videoDecoder;
};

int main() {
  VideoApp app;
  // Set video file here
  auto videoFile = "C:/Users/Andres/Downloads/Lw Kt Edit 0103 Good 75Mbps 8K "
                   "360-4k-30fps-noaudio.m4v";
  //  auto videoFile =
  //      "C:/Users/Andres/Downloads/LW_KT_Edit_0103_good_75mbps_8k_360.mp4";

  if (!app.loadVideoFile(videoFile)) {
    return -1;
  }
  app.start();
  return 0;
}
