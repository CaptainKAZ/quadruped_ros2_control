//
// Created by neo on 11/30/21.
//
/**
 * @brief: a socketcan wrapper
 */
#pragma once
#include <functional>
#include <linux/can.h>
#include <mutex>
#include <net/if.h>
#include <pthread.h>
#include <queue>
#include <string>

class SocketCan {
 public:
  SocketCan() = default;
  SocketCan(const std::string &interfaceName, uint32_t bitrate) {
    open(interfaceName, bitrate);
  }
  ~SocketCan();
  void open(const std::string &interfaceName, uint32_t bitrate);
  bool isOpen() const;
  void bindRxCallback(
      const std::function<bool(struct can_frame &, SocketCan *)> &callback);
  void clearRxCallback();
  void close();
  const SocketCan *operator>>(struct can_frame &frame);
  const SocketCan *operator<<(struct can_frame &frame);
  std::string interfaceName_;
 private:
  int sfd_ = -1;
  std::vector<std::function<bool(struct can_frame &, SocketCan *)>>
      rxCallbacks_;
  [[noreturn]] static void* rxThread(void *argv);
  [[noreturn]] static void* txThread(void *argv);
  std::queue<struct can_frame> rxQueue_;
  std::queue<struct can_frame> txQueue_;
  std::mutex rxMutex_;
  std::mutex txMutex_;
  pthread_t rxThreadHandle_ = 0;
  pthread_t txThreadHandle_ = 0;
};