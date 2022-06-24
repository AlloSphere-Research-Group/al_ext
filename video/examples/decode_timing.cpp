#include <chrono>
#include <cstdio>
#include <map>

#include "al_ext/video/al_VideoTexture.hpp"

int main() {

  VideoTexture videoDecoder{false};
  videoDecoder.load(

      //"C:/Users/Andres/Downloads/LW_KT_Edit_1205_360-convert.mp4"
      //      "C:/Users/Andres/Downloads/Lw Kt Edit 0103 Good 75Mbps 8K "
      //      "360-4k-30fps-noaudio.m4v"
      //      "C:/Users/Andres/Videos/Lw Kt Edit 0103 Good 75Mbps 8K
      //      360-Reencoded.m4v"

      //      "C:/Users/Andres/Videos/Lw Kt Edit 0103 Good 75Mbps 8K "
      //      "360-Reencoded-keyint60.m4v"
      "C:/Users/Andres/Videos/Lw Kt Edit 0103 4K 360-Reencoded-quality38.m4v"

  );
  std::map<int, int> timeHistogram;
  uint64_t sum = 0;
  uint64_t count = 0;
  uint64_t max = 0;
  uint64_t min = UINT64_MAX;

  videoDecoder.setDryRun(true);
  videoDecoder.start();

  int failedConsecutive = 0;

  while (true) {
    auto start = std::chrono::high_resolution_clock::now();
    uint8_t *frame;
    frame = videoDecoder.getVideoFrame(-1);
    while (!frame && failedConsecutive < 100) {
      failedConsecutive++;
      frame = videoDecoder.getVideoFrame(-1);
    }
    failedConsecutive = 0;
    videoDecoder.releaseVideoFrame();

    auto delta = std::chrono::high_resolution_clock::now() - start;
    auto deltaCount =
        std::chrono::duration_cast<std::chrono::microseconds>(delta).count();
    timeHistogram[deltaCount]++;
    count++;
    sum += deltaCount;
    if (deltaCount < min && deltaCount > 10) {
      min = deltaCount;
    }
    if (deltaCount > max) {
      max = deltaCount;
    }
    std::cout << "Frame " << count << " | Min time: " << min
              << " | Max time: " << max
              << " | Average time: " << (float)sum / count << "\r";
  }
  return 0;
}
