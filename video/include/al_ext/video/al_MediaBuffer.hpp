#ifndef AL_MEDIABUFFER_HPP
#define AL_MEDIABUFFER_HPP

// https://github.com/Golim4r/OpenGL-Video-Player

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <vector>

struct MediaFrame {
  MediaFrame() {}
  MediaFrame(uint8_t *data_ptr, size_t data_size, double data_pts)
      : dataY(data_ptr, data_ptr + data_size), pts(data_pts) {}
  MediaFrame(uint8_t *dataY_ptr, size_t dataY_size, uint8_t *dataU_ptr,
             size_t dataU_size, uint8_t *dataV_ptr, size_t dataV_size,
             double data_pts)
      : dataY(dataY_ptr, dataY_ptr + dataY_size),
        dataU(dataU_ptr, dataU_ptr + dataU_size),
        dataV(dataV_ptr, dataV_ptr + dataV_size), pts(data_pts) {}

  void clear() {
    dataY.clear();
    dataU.clear();
    dataV.clear();
    pts = 0;
  }

  std::vector<uint8_t> dataY;
  std::vector<uint8_t> dataU;
  std::vector<uint8_t> dataV;
  double pts;
};

class MediaBuffer {
public:
  MediaBuffer(int numElements)
      : frames(numElements), valid(numElements), readPos(0), writePos(0) {}

  bool put(uint8_t *buffer, int &numBytes, double &pts) {
    if (valid[writePos]) {
      // std::cerr << "Buffer pos already occupied: " << writePos << std::endl;
      return false;
    }

    // std::cout << "Writing at buffer: " << writePos << std::endl;

    // TODO: check possible unnecessary memory copy
    frames[writePos] = MediaFrame(buffer, numBytes, pts);
    valid[writePos] = true;
    writePos = ++writePos % frames.size();

    return true;
  }

  bool put(uint8_t *bufferY, int &numBytesY, uint8_t *bufferU, int &numBytesU,
           uint8_t *bufferV, int &numBytesV, double &pts) {
    if (valid[writePos]) {
      // std::cerr << "Buffer pos already occupied: " << writePos << std::endl;
      return false;
    }

    // std::cout << "Writing at buffer: " << writePos << std::endl;

    // TODO: check possible unnecessary memory copy
    frames[writePos] = MediaFrame(bufferY, numBytesY, bufferU, numBytesU,
                                  bufferV, numBytesV, pts);
    valid[writePos] = true;
    writePos = ++writePos % frames.size();

    return true;
  }

  MediaFrame *get() {
    if (!valid[readPos]) {
      // std::cerr << " Buffer pos empty: " << readPos << std::endl;
      cond.notify_one();
      return nullptr;
    }

    // std::cout << " Reading at buffer: " << readPos << std::endl;
    return &frames[readPos];
  }

  void got() {
    valid[readPos] = false;
    readPos = ++readPos % frames.size();
    cond.notify_one();
  }

  void flush() {
    // std::cout << "*** flushing : " << frames.size() << std::endl;
    for (int i = 0; i < frames.size(); ++i) {
      valid[i] = false;
      // frames[i].clear();
    }

    readPos = 0;
    writePos = 0;

    cond.notify_one();
  }

  int getReadPos() { return readPos.load(); }

  int getWritePos() { return writePos.load(); }

  std::mutex mutex;
  std::condition_variable cond;

private:
  std::vector<MediaFrame> frames;
  std::vector<std::atomic<bool>> valid;

  std::atomic<int> readPos;
  std::atomic<int> writePos;
};

#endif