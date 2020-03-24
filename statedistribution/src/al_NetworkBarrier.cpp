#include "al_ext/statedistribution/al_NetworkBarrier.hpp"

#include <array>

using namespace al;

namespace Convert {
auto to_bytes(std::uint16_t x) {
  std::array<std::uint8_t, 2> b;
  b[0] = x >> 8 * 0;
  b[1] = x >> 8 * 1;
  /*
 b[2] = x >> 8 * 2;
 b[3] = x >> 8 * 3;
 b[4] = x >> 8 * 4;
 b[5] = x >> 8 * 5;
 b[6] = x >> 8 * 6;
 b[7] = x >> 8 * 7;*/
  return b;
}

auto to_bytes(std::uint32_t x) {
  std::array<std::uint8_t, 4> b;
  b[0] = x >> 8 * 0;
  b[1] = x >> 8 * 1;
  b[2] = x >> 8 * 2;
  b[3] = x >> 8 * 3;
  /*b[4] = x >> 8 * 4;
 b[5] = x >> 8 * 5;
 b[6] = x >> 8 * 6;
 b[7] = x >> 8 * 7;*/
  return b;
}

void from_bytes(const uint8_t *bytes, uint16_t &dest) {
  dest = (uint16_t(bytes[1]) << 8 * 1) | (uint16_t(bytes[0]) << 8 * 0);
}

void from_bytes(const uint8_t *bytes, uint32_t &dest) {
  dest = (uint16_t(bytes[3]) << 8 * 3) | (uint16_t(bytes[4]) << 8 * 2) |
         (uint16_t(bytes[1]) << 8 * 1) | (uint16_t(bytes[0]) << 8 * 0)

      ;
}

} // namespace Convert

bool NetworkBarrier::initServer(uint16_t port, const char *addr) {
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
  mState = BarrierState::SERVER;
  mRunning = true;
  mServerThread = std::make_unique<std::thread>([&]() {
    // Receive data
    std::cout << "Server started" << std::endl;

    while (mRunning) {
      Socket bootstrapSocket;
      if (mSocket.accept(bootstrapSocket)) {
        std::cout << "Got Connection Request " << bootstrapSocket.address()
                  << ":" << bootstrapSocket.port() << std::endl;
        char message[8];
        int bytesRecv = bootstrapSocket.recv(message, 8);
        if (bytesRecv == 8) {
          if (message[0] == 1 << COMMAND_HANDSHAKE) {
            uint16_t port;
            Convert::from_bytes((const uint8_t *)&message[1], port);

            std::cout << "Handshake for " << bootstrapSocket.address() << ":"
                      << port << std::endl;

            mConnectionsLock.lock();
            mServerConnections.emplace_back(std::make_shared<Socket>());

            auto s = mServerConnections.back();
            s->open(port, bootstrapSocket.address().c_str(), 5, Socket::TCP);
            if (!s->connect()) {
              std::cerr << "ERROR establishing connection to client"
                        << std::endl;
              mServerConnections.resize(mServerConnections.size() - 1);
            }
            mConnectionsLock.unlock();
          }
        }
        bootstrapSocket.close();
      }
    }

    std::cout << "Server quit" << std::endl;
  });
  return true;
}

bool NetworkBarrier::initClient(uint16_t serverPort, const char *serverAddr) {
  Socket bootstrapSocket;
  if (!bootstrapSocket.open(serverPort, serverAddr, 1, Socket::TCP)) {
    std::cerr << "Error opening bootstrap socket" << std::endl;
    return false;
  }
  if (!bootstrapSocket.connect()) {
    std::cerr << "Error connecting bootstrap socket" << std::endl;
    return false;
  }
  int maxPorts = 1000;
  uint16_t portToTry = mPortOffset;

  {
    std::unique_lock<std::mutex> lk(mConnectionsLock);
    mServerConnections.push_back(std::make_shared<Socket>());
  }
  auto s = mServerConnections.back();

  while (portToTry++ < mPortOffset + maxPorts) {
    // TODO allow configuring on which network deveices to set up this
    // communication
    if (!s->open(portToTry, "0.0.0.0", 0.5, Socket::TCP)) {
      std::cerr << "ERROR on client listener open" << std::endl;
      continue;
    }
    if (!s->bind()) {
      s->close();
      std::cerr << "ERROR on client listener bind" << std::endl;
      continue;
    }

    if (!s->listen()) {
      s->close();
      std::cerr << "ERROR on client listener listen" << std::endl;
      continue;
    }

    break;
  }

  if (!s->opened()) {
    std::cerr << "Error opening client listener" << std::endl;
    std::unique_lock<std::mutex> lk(mConnectionsLock);
    mServerConnections.resize(mServerConnections.size() - 1);
    return false;
  }

  mState = BarrierState::CLIENT;
  mRunning = true;

  mConnectionThreads.emplace_back(std::make_unique<std::thread>(
      [&](std::shared_ptr<Socket> client) {
        Socket connectionSocket;
        client->timeout(5);
        connectionSocket.timeout(5);
        while (mRunning && !client->accept(connectionSocket)) {
          std::cout << "Waiting for server connection "
                    << connectionSocket.port() << std::endl;
        }
        if (!mRunning) {
          return;
        }
        std::cout << "Client listening port " << connectionSocket.port()
                  << std::endl;
        connectionSocket.timeout(0.01);
        while (mRunning) {
          unsigned char commandMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};
          size_t bytes = connectionSocket.recv((char *)commandMessage, 8);
          if (bytes == 8) {
            if (commandMessage[0] == 1 << COMMAND_PING) {
              clientHandlePing(connectionSocket);
            } else if (commandMessage[0] == 1 << COMMAND_PONG) {
              clientHandlePong(connectionSocket);
            } else if (commandMessage[0] == 1 << COMMAND_TRIGGER) {
              clientHandleTrigger(connectionSocket);
            } else if (commandMessage[0] == 1 << COMMAND_SYNC_REQ) {
              uint32_t id;
              Convert::from_bytes(&commandMessage[1], id);
              clientHandleSyncReq(connectionSocket, id);
            } else if (commandMessage[0] == 1 << COMMAND_BARRIER_LOCK) {
              std::cout << "Barrier lock not implemented " << std::endl;
            } else if (commandMessage[0] == 1 << COMMAND_BARRIER_UNLOCK) {
              uint32_t id;
              Convert::from_bytes(&commandMessage[1], id);
              clientHandleUnlock(connectionSocket, id);
            } else {
              std::cout << "Could not process command "
                        << (int)commandMessage[0] << std::endl;
            }
          } else if (bytes != 0 && bytes != (size_t)-1) {
            std::cerr << "ERROR unexpected command size " << bytes << std::endl;
            mRunning = false;
          }
        }
        connectionSocket.close();
        std::cout << "Client stopped " << std::endl;
        reset();
        std::unique_lock<std::mutex> lk(mConnectionsLock);
        mServerConnections.erase(std::find_if(
            mServerConnections.begin(), mServerConnections.end(),
            [&](std::shared_ptr<Socket> const &p) { return p == client; }));
      },
      s));

  unsigned char message[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // TODO is there a practical need to support big-endian?
  message[0] = 1 << COMMAND_HANDSHAKE;
  auto b = Convert::to_bytes(s->port());
  message[1] = b[0];
  message[2] = b[1];

  auto bytesSent = bootstrapSocket.send((const char *)message, 8);
  if (bytesSent != 8) {
    std::cerr << "ERROR sending handshake" << std::endl;
  }

  return true;
}

void NetworkBarrier::cleanup() {
  mRunning = false;
  mSocket.close();
  if (mServerThread) {
    mServerThread->join();
  }
  mConnectionsLock.lock();
  for (auto connectionSocket : mServerConnections) {
    connectionSocket->close();
  }

  mConnectionsLock.unlock();
  for (auto &connection : mConnectionThreads) {
    connection->join();
  }
  mState = BarrierState::NONE;
}

bool NetworkBarrier::pingClients(double timeoutSecs) {
  if (mState != BarrierState::SERVER) {
    std::cerr << "ERROR: Should not call ping from server" << std::endl;
    return false;
  }

  bool allResponded = true;

  mConnectionsLock.lock();
  for (auto listener : mServerConnections) {
    std::cout << "pinging " << listener->address() << ":" << listener->port()
              << std::endl;
    auto startTime = al_steady_time();
    unsigned char message[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    message[0] = 1 << COMMAND_PING;
    listener->send((const char *)message, 8);
    size_t bytes = 0;
    auto previousTimeout = listener->timeout();
    listener->timeout(timeoutSecs);
    bytes = listener->recv((char *)message, 8);

    auto endTime = al_steady_time();
    if (bytes == 8) {
      std::cout << "Reply from " << listener->address() << ":"
                << listener->port() << " in " << (endTime - startTime) * 1000.0
                << " ms" << std::endl;
    } else {
      std::cout << "No response from: " << listener->address() << ":"
                << listener->port() << std::endl;
    }
    listener->timeout(previousTimeout);
  }

  mConnectionsLock.unlock();

  return allResponded;
}

bool NetworkBarrier::trigger(uint32_t id, double waitTimeoutSecs) {
  if (mState == BarrierState::SERVER) {
    mConnectionsLock.lock();
    for (auto listener : mServerConnections) {
      //      std::cout << "synchronizing " << listener->address() << ":"
      //                << listener->port() << std::endl;
      auto startTime = al_steady_time();
      unsigned char message[8] = {0, 0, 0, 0, 0, 0, 0, 0};

      message[0] = 1 << COMMAND_TRIGGER;

      auto b = Convert::to_bytes(id);
      message[1] = b[0];
      message[2] = b[1];
      message[3] = b[2];
      message[4] = b[3];
      listener->send((const char *)message, 8);
      //      size_t bytes = 0;
      //      auto previousTimeout = listener->timeout();
      //      listener->timeout(waitTimeoutSecs);
      //      bytes = listener->recv((char *)message, 8);

      //      auto endTime = al_steady_time();
      //      if (bytes == 8) {
      //        uint32_t id;
      //        Convert::from_bytes((const uint8_t *)&message[1], id);

      //        //        std::cout << "Synced: " << listener->address() << ":"
      //        //                  << listener->port() << " id " << id <<
      //        std::endl;
      //      } else {
      //        std::cout << "No response from: " << listener->address() << ":"
      //                  << listener->port() << std::endl;
      //      }
      //      listener->timeout(previousTimeout);
    }

    mConnectionsLock.unlock();

  } else if (mState == BarrierState::CLIENT) {
    //    std::cout << "sync lock" << std::endl;
    if (mClientMessageLock.find(id) == mClientMessageLock.end()) {
      mClientMessageLock[id] = {std::make_unique<std::mutex>(),
                                std::make_unique<std::condition_variable>()};
    }
    auto &lock = mClientMessageLock[id].first;
    auto &conditionVar = mClientMessageLock[id].second;
    std::unique_lock<std::mutex> lk(*lock);
    conditionVar->wait(lk);
    //    std::cout << "sync continue" << std::endl;
  }
  return false;
}

bool NetworkBarrier::synchronize(uint32_t id, double waitTimeoutSecs) {
  if (mState == BarrierState::SERVER) {
    mConnectionsLock.lock();
    unsigned char message[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // Notify all
    for (auto listener : mServerConnections) {
      //      std::cout << "synchronizing " << listener->address() << ":"
      //                << listener->port() << std::endl;
      //      auto startTime = al_steady_time();

      message[0] = 1 << COMMAND_SYNC_REQ;

      auto b = Convert::to_bytes(id);
      message[1] = b[0];
      message[2] = b[1];
      message[3] = b[2];
      message[4] = b[3];
      auto previousTimeout = listener->timeout();
      size_t bytes = listener->send((const char *)message, 8);
      listener->timeout(previousTimeout);
      if (bytes != 8) {
        std::cerr << "Error sending to " << listener->address() << ":"
                  << listener->port() << std::endl;
      }
    }

    // Get reply from all
    for (auto listener : mServerConnections) {
      size_t bytes = 0;
      auto previousTimeout = listener->timeout();
      listener->timeout(waitTimeoutSecs);
      bytes = listener->recv((char *)message, 8);

      //      auto endTime = al_steady_time();
      if (bytes == 8 && message[0] == 1 << COMMAND_SYNC_ACK) {
        uint32_t ackid;
        Convert::from_bytes((const uint8_t *)&message[1], ackid);
        if (ackid == id) {
          std::cout << "Synced: " << listener->address() << ":"
                    << listener->port() << " id " << id << std::endl;
        } else {
          std::cerr << "Unexpected ack id" << std::endl;
        }

      } else {
        std::cout << "No response from: " << listener->address() << ":"
                  << listener->port() << std::endl;
      }
      listener->timeout(previousTimeout);
    }
    // Unlock all
    for (auto listener : mServerConnections) {
      //      std::cout << "synchronizing " << listener->address() << ":"
      //                << listener->port() << std::endl;
      //      auto startTime = al_steady_time();

      message[0] = 1 << COMMAND_BARRIER_UNLOCK;

      auto b = Convert::to_bytes(id);
      message[1] = b[0];
      message[2] = b[1];
      message[3] = b[2];
      message[4] = b[3];
      auto previousTimeout = listener->timeout();
      size_t bytes = listener->send((const char *)message, 8);
      listener->timeout(previousTimeout);
      if (bytes != 8) {
        std::cerr << "Error sending to " << listener->address() << ":"
                  << listener->port() << std::endl;
      }
    }

    mConnectionsLock.unlock();

  } else if (mState == BarrierState::CLIENT) {
    //    std::cout << "sync lock" << std::endl;
    if (mRunning) {
      if (mClientMessageLock.find(id) == mClientMessageLock.end()) {

        mClientMessageLock[id] = {std::make_unique<std::mutex>(),
                                  std::make_unique<std::condition_variable>()};
      }
      auto &lock = mClientMessageLock[id].first;
      auto &conditionVar = mClientMessageLock[id].second;
      std::unique_lock<std::mutex> lk(*lock);
      if (waitTimeoutSecs > 0) {
        conditionVar->wait_for(
            lk, std::chrono::milliseconds((int)(waitTimeoutSecs * 1000.0)));
      } else {
        conditionVar->wait(lk);
      }
    }
    //    std::cout << "sync continue" << std::endl;
  }
  return false;
}

uint16_t NetworkBarrier::waitForConnections(uint16_t connectionCount,
                                            double timeout) {
  if (mState == BarrierState::SERVER) {
    double targetTime = al_steady_time() + timeout;
    double currentTime = al_steady_time();

    size_t existingConnections = mServerConnections.size();
    mConnectionsLock.lock();
    size_t totalConnections = mServerConnections.size();
    mConnectionsLock.unlock();
    while (targetTime >= currentTime) {
      mConnectionsLock.lock();
      // TODO what should happen if there are disconnections instead of
      // connections while this runs?
      totalConnections = mServerConnections.size();
      mConnectionsLock.unlock();
      // FIXME this could allow more connections through than requested. Should
      // the number be treated as a maximum?
      if (totalConnections - existingConnections < connectionCount) {
        al_sleep(0.3);
      } else {
        return totalConnections - existingConnections;
      }
      currentTime = al_steady_time();
    }
    return totalConnections - existingConnections;
  } else {
    // TODO make sure clients connect
  }
  return 0;
}

size_t NetworkBarrier::connectionCount() {
  mConnectionsLock.lock();
  size_t numConnections = mServerConnections.size();
  mConnectionsLock.unlock();
  return numConnections;
}

bool NetworkBarrier::barrier(uint32_t id, double waitTimeoutSecs) {
  if (mState == BarrierState::SERVER) {
    mConnectionsLock.lock();

    // First lock and wait for all to acknowledge lock
    for (auto listener : mServerConnections) {
      //      std::cout << "synchronizing " << listener->address() << ":"
      //                << listener->port() << std::endl;
      auto startTime = al_steady_time();
      unsigned char message[8] = {0, 0, 0, 0, 0, 0, 0, 0};

      message[0] = 1 << COMMAND_BARRIER_LOCK;

      auto b = Convert::to_bytes(id);
      message[1] = b[0];
      message[2] = b[1];
      message[3] = b[2];
      message[4] = b[3];
      listener->send((const char *)message, 8);
      size_t bytes = 0;
      auto previousTimeout = listener->timeout();
      listener->timeout(waitTimeoutSecs);
      bytes = listener->recv((char *)message, 8);

      auto endTime = al_steady_time();
      if (bytes == 8) {
        uint32_t id;
        Convert::from_bytes((const uint8_t *)&message[1], id);

        std::cout << "Synced: " << listener->address() << ":"
                  << listener->port() << " id " << id << std::endl;
      } else {
        std::cout << "No response from: " << listener->address() << ":"
                  << listener->port() << std::endl;
      }
      listener->timeout(previousTimeout);
    }

    // Then unlock all
    for (auto listener : mServerConnections) {
      auto startTime = al_steady_time();
      unsigned char message[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      message[0] = 1 << COMMAND_BARRIER_UNLOCK;

      auto b = Convert::to_bytes(id);
      message[1] = b[0];
      message[2] = b[1];
      message[3] = b[2];
      message[4] = b[3];
      if (listener->send((const char *)message, 8) != 8) {
        std::cerr << "ERROR sending unlock to " << listener->address() << ":"
                  << listener->port() << std::endl;
      }
    }
    // Now check for unlock acknowledge
    for (auto listener : mServerConnections) {
      unsigned char message[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      auto previousTimeout = listener->timeout();
      listener->timeout(0);
      size_t bytes = listener->recv((char *)message, 8);

      if (bytes != 8) {
        std::cerr << "ERROR receiving unlock ack from " << listener->address()
                  << ":" << listener->port() << std::endl;
      }
      listener->timeout(previousTimeout);
    }

    mConnectionsLock.unlock();
  } else if (mState == BarrierState::CLIENT) {
    //    std::cout << "sync lock" << std::endl;
    if (mClientMessageLock.find(id) == mClientMessageLock.end()) {

      mClientMessageLock[id] = {std::make_unique<std::mutex>(),
                                std::make_unique<std::condition_variable>()};
    }
    auto &lock = mClientMessageLock[id].first;
    auto &conditionVar = mClientMessageLock[id].second;
    std::unique_lock<std::mutex> lk(*lock);
    conditionVar->wait(lk);
    //    std::cout << "sync continue" << std::endl;
  }
  return false;
}

void NetworkBarrier::reset() {
  for (auto &lock : mClientMessageLock) {
    auto &conditionVar = lock.second.second;
    std::unique_lock<std::mutex> lk(*lock.second.first);
    conditionVar->notify_all();
  }
}

void NetworkBarrier::clientHandlePing(Socket &client) {
  //    std::cout << "got ping request" << std::endl;
  char buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  buffer[0] = 1 << COMMAND_PONG;
  //  std::cout << "sending pong" << std::endl;
  int bytesSent = client.send((const char *)buffer, 8);
  if (bytesSent != 8) {
    std::cerr << "ERROR: sent bytes mismatch for pong" << std::endl;
  }
}

void NetworkBarrier::clientHandlePong(Socket &client) {
  std::cout << "got pong request. Ignoring" << std::endl;
}

void NetworkBarrier::clientHandleTrigger(Socket &client) {

  std::cout << "Trigger" << std::endl;
}

void NetworkBarrier::clientHandleSyncReq(Socket &client, uint32_t id) {
  char buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  buffer[0] = 1 << COMMAND_SYNC_ACK;
  std::cout << "sending sync ack id " << id << std::endl;
  int bytesSent = client.send((const char *)buffer, 8);
  if (bytesSent != 8) {
    std::cerr << "ERROR: sent bytes mismatch for sync ack" << std::endl;
  }
}

void NetworkBarrier::clientHandlSyncAck(Socket &client) {
  std::cout << "got sync ack. Ignoring" << std::endl;
}

void NetworkBarrier::clientHandleUnlock(Socket &client, uint32_t id) {
  std::cout << "Unlocking id " << id << std::endl;
  auto &lock = mClientMessageLock[id].first;
  auto &conditionVar = mClientMessageLock[id].second;
  std::unique_lock<std::mutex> lk(*lock);
  conditionVar->notify_one();
}
