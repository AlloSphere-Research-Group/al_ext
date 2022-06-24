#include <cstdio>

#include "al/app/al_App.hpp"
#include "al/app/al_GUIDomain.hpp"
#include "al/graphics/al_Shapes.hpp"

#include "al/graphics/al_Font.hpp"
#include "al_ext/video/al_VideoTexture.hpp"

class VideoApp : public App {
public:
  ParameterBool freeWheel{"freeWheel", "", true};

  double time;
  double wallClock;
  double frameDelta;
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> previousTime;

  virtual void onInit() override {
    // FPS can't be set on onCreate(), has to be set here
    fps(videoDecoder.fps());
    aspectRatio = videoDecoder.width() / (double)videoDecoder.height();

    time = 0;
    frameDelta = 0;
  }
  virtual void onCreate() override {
    startTime = std::chrono::high_resolution_clock::now();
    videoDecoder.start();

    auto guiDomain = GUIDomain::enableGUI(defaultWindowDomain());
    auto panel = guiDomain->newPanel();
    panel->gui << freeWheel;
  }

  virtual void onAnimate(al_sec dt) override {
    //    std::cout << "requesting frame at " << time << " delta: " <<
    //    frameDelta
    //              << "  dt: " << dt << std::endl;
    if (freeWheel) {
      videoDecoder.updateTexture(-1);
    } else {
      videoDecoder.updateTexture(time);
    }
    time += dt;

    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = now - startTime;
    wallClock = diff.count();
    std::chrono::duration<double> frameDiff = now - previousTime;
    frameDelta = frameDiff.count();
    previousTime = now;
  }

  std::string toTimeCode(double time) {
    char buf[14];
    float whole, fractional;

    fractional = std::modf(time, &whole);
    int frames = fractional * 30;
    int seconds = int(time);
    int minutes = seconds / 60;
    int hours = minutes / 60;
    sprintf(buf, "%02i:%02i:%02i:%02i", hours % 24, minutes % 60, seconds % 60,
            frames);
    return buf;
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

    auto diff = videoDecoder.getCurrentFrameTime() - time;
    std::string text = toTimeCode(videoDecoder.getCurrentFrameTime()) + "  " +
                       toTimeCode(time) + "   " + toTimeCode(diff);
    FontRenderer::render(g, text.c_str(), Vec3d(-0.33, -0.2, -1), 0.03, 24);

    text = toTimeCode(wallClock) + "   " + std::to_string(frameDelta);
    FontRenderer::render(g, text.c_str(), Vec3d(-0.33, -0.23, -1), 0.03, 24);
  }

  bool loadVideoFile(std::string videoFileUrl) {
    if (!videoDecoder.load(videoFileUrl.c_str())) {
      std::cerr << "Error loading video file" << std::endl;
      return false;
    }
    return true;
  }

  bool onKeyDown(Keyboard const &k) override {
    if (k.key() == ']') {
      videoDecoder.seek(videoDecoder.getCurrentFrameTime() + 5.0);
    }
    return true;
  }

  double aspectRatio{1.0};

private:
  VideoTexture videoDecoder;
  std::string mVideoFileToLoad;
};

int main() {
  VideoApp app;
  // Set video file here
  //  auto videoFile = "C:/Users/Andres/Downloads/Lw Kt Edit 0103 Good 75Mbps 8K
  //  "
  //                   "360-4k-30fps-noaudio.m4v";
  //  auto videoFile =
  //      "C:/Users/Andres/Downloads/LW_KT_Edit_0103_good_75mbps_8k_360.mp4";
  //  auto videoFile = "C:/Users/Andres/Downloads/LW_KT_Edit_1205_360.mp4";

  //  auto videoFile = "C:/Users/Andres/Downloads/Lw Kt Edit 0103 Good 75Mbps 8K
  //  "
  //                   "360-4k-30fps-noaudio.m4v";

  //  auto videoFile =
  //      "C:/Users/Andres/Videos/Lw Kt Edit 0103 Good 75Mbps 8K
  //      360-Reencoded.m4v";

  auto videoFile = "C:/Users/Andres/Videos/Lw Kt Edit 0103 Good 75Mbps 8K "
                   "360-Reencoded-keyint60.m4v";
  if (!app.loadVideoFile(videoFile)) {
    return -1;
  }

  app.start();
  return 0;
}
