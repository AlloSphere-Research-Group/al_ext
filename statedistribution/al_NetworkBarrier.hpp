#ifndef AL_NETWORKBARRIER_HPP
#define AL_NETWORKBARRIER_HPP

/* Andres Cabrera, 2019, mantaraya36@gmail.com
 */

#include <cinttypes>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "al/io/al_Socket.hpp"
#include "al/types/al_SingleRWRingBuffer.hpp"

namespace al {

class NetworkBarrier {
 public:
  enum { COMMAND_PING };

  bool initServer(uint16_t port, const char *addr) {
    if (!mSocket.open(port, addr, 0.5, Socket::TCP)) {
      std::cerr << "ERROR opening port" << std::endl;
      return false;
    }
    if (!mSocket.bind()) {
      std::cerr << "ERROR on bind" << std::endl;
      return false;
    }

    if (!mSocket.listen()) {
      std::cerr << "ERROR on listen" << std::endl;
      return false;
    }
    mRunning = true;
    connections.emplace_back(
        std::make_shared<Socket>());  // Start with one free socket.
    mServerThread = std::make_unique<std::thread>([&]() {
      // Receive data
      std::cout << "Server started" << std::endl;
      // Typically, a server will loop to check for incoming packets
      bool connected = false;

      while (mRunning) {
        if (!connected) {
          auto s = connections.back();
          if (mSocket.accept(*s)) {
            connected = true;
            connectionThreads.emplace_back(std::make_unique<std::thread>(
                [&](std::shared_ptr<Socket> s) {
                  std::cout << "Got Connection " << (long)s.get() << std::endl;
                  while (mRunning) {
                    char buf[128];
                    int bytesRecv = s->recv(buf, 128);
                    if (bytesRecv >= 0) {
                      std::cout << "received " << bytesRecv << std::endl;
                      s->send("1234\0", 5);
                    }
                  }
                  std::cout << "Closed Connection " << (long)s.get()
                            << std::endl;
                },
                s));

            connections.emplace_back(std::make_shared<Socket>());
          }
        } else {
        }
      }

      std::cout << "Server quit" << std::endl;
    });
    return true;
  }

  bool initClient(uint16_t port, const char *addr) {
    if (!mSocket.open(port, addr, 0.3, Socket::TCP)) {
      return false;
    }
    if (!mSocket.connect()) {
      return false;
    }

    connectionThreads.emplace_back(std::make_unique<std::thread>(
        [&](Socket *client) {
          std::cout << "Client started " << std::endl;
          while (mRunning) {
            //            std::unique_lock<std::mutex> lk(mClientMessageLock);
            unsigned char message[4];
            size_t commandBytes = commandSendBuffer.read((char *)message, 4);
            if (commandBytes == 4) {
              std::cout << "Got command " << (int)message[0] << std::endl;
              if (message[0] == 1 << COMMAND_PING) {
                int bytesSent = client->send("1234\0", 5);
                std::cout << "sent " << bytesSent << std::endl;
                char buffer[128];
                int received = client->recv(buffer, 128);
                std::cout << "rec " << received << std::endl;
              }
            } else if (commandBytes != 0) {
              std::cerr << "ERROR: Internal command buffer underrun"
                        << std::endl;
            }
          }

          std::cout << "Client stopped " << std::endl;
        },
        &mSocket));

    return true;
  }

  void cleanup() {
    mRunning = false;
    mSocket.close();
    if (mServerThread) {
      mServerThread->join();
    }
    for (auto connectionSocket : connections) {
      connectionSocket->close();
    }
    for (auto &connection : connectionThreads) {
      connection->join();
    }
  }

  /**
   * @brief Wait for timeoutSecs for reply from all.
   * @param timeoutSecs
   * @return true if all nodes replied by timeout
   *
   * On primary, ping is sent and wait for replies. On secondary, this line
   * is ignored. If timeout is 0, this function blocks until all replies are
   * received or
   */
  bool ping(double timeoutSecs = 0.0) {
    unsigned char message[4] = {0, 0, 0, 0};
    message[0] = 1 << COMMAND_PING;
    commandSendBuffer.write((const char *)message, 4);

    return true;
  }

  /**
   * @brief Block and wait for all nodes to respond
   *
   * Primary will issue a network barrier command and wait for all nodes to
   * acknowledge. Returns true of synchronized, false on timeout
   */
  bool synchronize(uint32_t id = 0, double waitTimeoutSecs = 0.0);

  /**
   * @brief Block until received a wakeup signal
   * @return true if signal received, flase if timedout
   *
   * This blocks only on secondary nodes. It is the primary's job to
   * call signal() to wake up secondary nodes.
   */
  bool barrier(uint32_t id = 0, double waitTimeoutSecs = 0.0);

  /**
   * @brief signal to wake up a barrier
   * @return true if all barriers were notified and replied before timeout
   */
  bool signal(uint32_t id = 0, double waitTimeoutSecs = 0.0);

  bool tickConsecutive(uint32_t id = 0);

  /**
   * @brief Resets this barrier and forces any pending sends or receives to be
   * cancelled
   */
  void reset();

 private:
  SocketServer mSocket;
  std::unique_ptr<std::thread> mServerThread;

  std::vector<std::unique_ptr<std::thread>> connectionThreads;
  std::vector<std::shared_ptr<Socket>> connections;

  std::mutex mClientMessageLock;

  SingleRWRingBuffer commandSendBuffer;

  bool mRunning;
};

}  // namespace al

#endif  // AL_NETWORKBARRIER_HPP
