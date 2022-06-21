#include <cstdio>

#include "al_ext/video/al_VideoTexture.hpp"

#include "gtest/gtest.h"

TEST(Video, BasicDecode) {
  VideoTexture videoDecoder{false};
  videoDecoder.load(

      "C:/Users/Andres/Downloads/Lw Kt Edit 0103 Good 75Mbps 8K "
      "360-4k-30fps-noaudio.m4v");
  //  videoDecoder.load(
  //      "C:/Users/Andres/Downloads/LW_KT_Edit_1205_360-convert.mp4");

  videoDecoder.start();

  EXPECT_EQ(videoDecoder.readFramesInBuffer(), 7);

  uint8_t *frame;

  auto frameInterval = 1.0 / videoDecoder.fps();

  for (int i = 0; i < 100; i++) {
    frame = videoDecoder.getVideoFrame();
    int count = 0;
    while (!frame && count < 50) {
      // Gice time for buffer to fill
      frame = videoDecoder.getVideoFrame();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      count++;
    }
    EXPECT_TRUE(frame);
    EXPECT_FLOAT_EQ(videoDecoder.getCurrentFrameTime(), i * frameInterval)
        << "**  for " << i;
    videoDecoder.releaseVideoFrame();
  }
}

TEST(Video, SeekForward) {
  VideoTexture videoDecoder{false};
  videoDecoder.load(

      "C:/Users/Andres/Downloads/Lw Kt Edit 0103 Good 75Mbps 8K "
      "360-4k-30fps-noaudio.m4v");
  //  videoDecoder.load(
  //      "C:/Users/Andres/Downloads/LW_KT_Edit_1205_360-convert.mp4");

  videoDecoder.start();

  EXPECT_EQ(videoDecoder.readFramesInBuffer(), 7);

  uint8_t *frame;

  auto frameInterval = 1.0 / videoDecoder.fps();

  std::vector<int> targetFrames = {5,   10,  30,  40,  51,  72,
                                   150, 152, 154, 156, 158, 500};
  //  std::vector<int> targetFrames = {5, 10, 30, 40, 51, 72, 150};

  for (const auto &tgtFrame : targetFrames) {
    frame = videoDecoder.getVideoFrame(tgtFrame * frameInterval);
    int count = 0;
    while (!frame && count < 50000) {
      // Give time for buffer to fill
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      frame = videoDecoder.getVideoFrame();
      count++;
    }

    EXPECT_TRUE(frame) << "Fail on frame: " << tgtFrame;
    EXPECT_FLOAT_EQ(videoDecoder.getCurrentFrameTime(),
                    tgtFrame * frameInterval)
        << "Fail on frame: " << tgtFrame;
    videoDecoder.releaseVideoFrame();
  }
}

TEST(Video, SeekBack) {
  VideoTexture videoDecoder{false};
  videoDecoder.load("C:/Users/Andres/Downloads/Lw Kt Edit 0103 Good 75Mbps 8K "
                    "360-4k-30fps-noaudio.m4v");
  //  videoDecoder.load(
  //      "C:/Users/Andres/Downloads/LW_KT_Edit_1205_360-convert.mp4");

  videoDecoder.start();

  EXPECT_EQ(videoDecoder.readFramesInBuffer(), 7);

  uint8_t *frame;

  auto frameInterval = 1.0 / videoDecoder.fps();

  std::vector<int> targetFrames = {500, 345, 240, 100, 72, 42,
                                   29,  24,  12,  6,   0};
  //  std::vector<int> targetFrames = {5, 10, 30, 40, 51, 72, 150};

  for (const auto &tgtFrame : targetFrames) {
    frame = videoDecoder.getVideoFrame(tgtFrame * frameInterval);
    int count = 0;
    while (!frame && count < 50000) {
      // Give time for buffer to fill
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      frame = videoDecoder.getVideoFrame();
      count++;
    }

    EXPECT_TRUE(frame) << "Fail on frame: " << tgtFrame;
    EXPECT_FLOAT_EQ(videoDecoder.getCurrentFrameTime(),
                    tgtFrame * frameInterval);
    videoDecoder.releaseVideoFrame();
  }
}

TEST(Video, SyncDecode) {
  VideoTexture videoDecoder{false};
  videoDecoder.load("C:/Users/Andres/Downloads/Lw Kt Edit 0103 Good 75Mbps 8K "
                    "360-4k-30fps-noaudio.m4v");
  //  videoDecoder.load(
  //      "C:/Users/Andres/Downloads/LW_KT_Edit_1205_360-convert.mp4");

  videoDecoder.start();

  EXPECT_EQ(videoDecoder.readFramesInBuffer(), 7);

  uint8_t *frame;

  auto frameInterval = 1.0 / videoDecoder.fps();

  for (int i = 0; i < 100; i++) {
    frame = videoDecoder.getVideoFrame(i * frameInterval);
    int count = 0;
    while (!frame && count < 50000) {
      // Gice time for buffer to fill
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      frame = videoDecoder.getVideoFrame();
      count++;
    }
    EXPECT_TRUE(frame);
    EXPECT_FLOAT_EQ(videoDecoder.getCurrentFrameTime(), i * frameInterval);
    videoDecoder.releaseVideoFrame();
  }
}
