#include <cstdio>

#include "al_ext/video/al_VideoTexture.hpp"

int main() {

  std::cout << "Press ']' to advance, '[' to rewind. Enter only for next frame"
            << std::endl;
  VideoTexture videoDecoder{false};
  videoDecoder.load("C:/Users/Andres/Downloads/Lw Kt Edit 0103 Good 75Mbps 8K "
                    "360-4k-30fps-noaudio.m4v");
  //  videoDecoder.load(
  //      "C:/Users/Andres/Downloads/LW_KT_Edit_1205_360-convert.mp4");

  videoDecoder.start();

  while (true) {
    auto c = getchar();
    uint8_t *frame;
    frame = videoDecoder.getVideoFrame(-1);

    if (frame) {
      if (videoDecoder.getCurrentFrameType() == AV_PICTURE_TYPE_I) {

        std::cout << "Keyframe at: " << videoDecoder.getCurrentFrameTime()
                  << " decoder time: " << videoDecoder.getCurrentDecoderTime()
                  << std::endl;
      }
      videoDecoder.releaseVideoFrame();
    }
  }
  return 0;
}
