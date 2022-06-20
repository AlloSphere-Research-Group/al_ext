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
    if (c == '[') {
      frame =
          videoDecoder.getVideoFrame(videoDecoder.getCurrentFrameTime() - 2.0);
    } else if (c == ']') {
      frame =
          videoDecoder.getVideoFrame(videoDecoder.getCurrentFrameTime() + 2.0);
    } else if (c == '0') {
      frame = videoDecoder.getVideoFrame(0.0);
    } else {
      frame = videoDecoder.getVideoFrame(-1);
    }
    if (frame) {
      std::cout << " read frame. frames in buffer: "
                << videoDecoder.readFramesInBuffer() << std::endl;
      std::cout << "frame time: " << videoDecoder.getCurrentFrameTime()
                << " decoder time: " << videoDecoder.getCurrentDecoderTime()
                << std::endl;
      videoDecoder.releaseVideoFrame();
    } else {
      std::cout << " no frame " << videoDecoder.readFramesInBuffer()
                << " decoder time: " << videoDecoder.getCurrentDecoderTime()
                << std::endl;
    }
  }
  return 0;
}
