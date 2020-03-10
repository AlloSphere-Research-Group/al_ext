#ifndef AL_NETWORKBARRIER_HPP
#define AL_NETWORKBARRIER_HPP

/* Andres Cabrera, 2019, mantaraya36@gmail.com
 */

#include <cinttypes>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "al/io/al_Socket.hpp"
#include "al/types/al_SingleRWRingBuffer.hpp"

namespace al {

class NetworkBarrier {
public:
  enum {
    COMMAND_PING,
    COMMAND_PONG,
    COMMAND_HANDSHAKE,
    COMMAND_TRIGGER,
    COMMAND_SYNC_REQ,
    COMMAND_SYNC_ACK,
    COMMAND_BARRIER_LOCK,
    COMMAND_BARRIER_UNLOCK,
    COMMAND_SERVER_SHUTODOWN
  };

  typedef enum { SERVER, CLIENT, NONE } BarrierState;

  bool initServer(uint16_t port = 10467, const char *addr = nullptr);

  bool initClient(uint16_t serverPort = 10467,
                  const char *serverAddr = nullptr);

  void cleanup();

  /**
   * @brief Wait for timeoutSecs for reply from all.
   * @param timeoutSecs
   * @return true if all nodes replied by timeout
   *
   * On primary, ping is sent and wait for replies. On secondary, this line
   * is ignored. If timeout is 0, this function blocks until all replies are
   * received or
   */
  bool pingClients(double timeoutSecs = 1.0);

  /**
   * @brief Block and notify all nodes
   *
   * Primary will issue a network barrier command and wait for all nodes to
   * receive the synchronize message. There is no wait for clients to
   * acknowledge. Returns true of synchronized, false on timeout
   */
  bool trigger(uint32_t id = 0, double waitTimeoutSecs = 0.0);

  /**
   * @brief Block and wait for all nodes to respond
   *
   * Primary will issue a network barrier command and wait for all nodes to
   * acknowledge. Returns true of synchronized, false on timeout
   */
  bool synchronize(uint32_t id = 0, double waitTimeoutSecs = 0.0);

  /**
   * @brief Block until connectionCount connections are established
   * @return number of connections acquired during wait
   */
  uint16_t waitForConnections(uint16_t connectionCount, double timeout = 60.0);

  size_t connectionCount();

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

  /**
   * @brief Resets this barrier and forces any pending sends or receives to be
   * cancelled
   */
  void reset();

private:
  Socket mSocket;
  std::unique_ptr<std::thread> mServerThread;

  std::vector<std::unique_ptr<std::thread>> mConnectionThreads;
  std::vector<std::shared_ptr<Socket>> mServerConnections;
  std::mutex mConnectionsLock;

  std::map<uint32_t, std::pair<std::unique_ptr<std::mutex>,
                               std::unique_ptr<std::condition_variable>>>
      mClientMessageLock;

  SingleRWRingBuffer mCommandSendBuffer;

  bool mRunning;

  BarrierState mState{BarrierState::NONE};

  uint16_t mPortOffset = 12000;

  void clientHandlePing(Socket &client);
  void clientHandlePong(Socket &client);
  void clientHandleTrigger(Socket &client);
  void clientHandleSyncReq(Socket &client, uint32_t id);
  void clientHandlSyncAck(Socket &client);
  void clientHandleUnlock(Socket &client, uint32_t id);
};

} // namespace al

#endif // AL_NETWORKBARRIER_HPP
