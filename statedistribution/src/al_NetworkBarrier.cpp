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

void from_bytes(const uint8_t *bytes, uint16_t &dest) {
  dest = (uint16_t(bytes[1]) << 8 * 1) | (uint16_t(bytes[0]) << 8 * 0);
}

}  // namespace Convert

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

            mServerConnections.emplace_back(std::make_shared<Socket>());

            auto s = mServerConnections.back();
            s->open(port, bootstrapSocket.address().c_str(), 5, Socket::TCP);
            if (!s->connect()) {
              std::cerr << "ERROR establishing connection to client"
                        << std::endl;
              mServerConnections.resize(mServerConnections.size() - 1);
            }
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

  mServerConnections.push_back(std::make_shared<Socket>());

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
    mServerConnections.resize(mServerConnections.size() - 1);
    return false;
  }

  mState = BarrierState::CLIENT;
  mRunning = true;

  mConnectionThreads.emplace_back(std::make_unique<std::thread>(
      [&](std::shared_ptr<Socket> client) {
        Socket connectionSocket;
        while (!client->accept(connectionSocket)) {
        }
        std::cout << "Client listening port " << connectionSocket.port()
                  << std::endl;

        while (mRunning) {
          unsigned char commandMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};
          size_t bytes = connectionSocket.recv((char *)commandMessage, 8);
          if (bytes == 8) {
            if (commandMessage[0] == 1 << COMMAND_PING) {
              clientHandlePing(connectionSocket);
            } else if (commandMessage[0] == 1 << COMMAND_PONG) {
              clientHandlePong(connectionSocket);
            } else {
              std::cout << "Could not process command "
                        << (int)commandMessage[0] << std::endl;
            }
          } else if (bytes != 0) {
            std::cerr << "ERROR unexpected command size " << bytes << std::endl;
          }
          al_sleep(0.01);
        }
        connectionSocket.close();
        std::cout << "Client stopped " << std::endl;
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
  for (auto connectionSocket : mServerConnections) {
    connectionSocket->close();
  }
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

  return allResponded;
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
