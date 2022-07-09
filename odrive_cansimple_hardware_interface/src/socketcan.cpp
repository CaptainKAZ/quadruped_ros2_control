//
// Created by neo on 11/30/21.
//

#include "odrive_cansimple_hardware_interface/socketcan.hpp"
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include "rcutils/logging_macros.h"

SocketCan::~SocketCan()
{
  if (this->isOpen())
  {
    close();
  }
}

void SocketCan::open(const std::string& interfaceName, const uint32_t bitrate)
{
  ifreq ifr{};
  sockaddr_can addr{};
  interfaceName_ = interfaceName;
  (void)system(("sudo ip link set " + interfaceName + " type can bitrate " + std::to_string(bitrate)).c_str());
  (void)system(("sudo ifconfig " + interfaceName + " up").c_str());
  (void)system(("sudo ip -details link show " + interfaceName).c_str());
  if ((sfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    RCUTILS_LOG_FATAL_NAMED("SocketCAN", "Can't open socket: %s", strerror(errno));
    exit(errno);
  }
  strncpy(ifr.ifr_name, interfaceName.c_str(), IFNAMSIZ);
  if (ioctl(sfd_, SIOCGIFINDEX, &ifr) < 0)
  {
    RCUTILS_LOG_FATAL_NAMED("SocketCAN", "Can't get interface index: %s", strerror(errno));
    exit(errno);
  }
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    RCUTILS_LOG_FATAL_NAMED("SocketCAN", "Can't bind socket: %s", strerror(errno));
    exit(errno);
  }
  std::cout << "Successfully open " << interfaceName << " at index " << ifr.ifr_ifindex << std::endl;
  pthread_attr_t can_thread_attr;
  struct sched_param can_thread_sched_param = {};
  can_thread_sched_param.sched_priority =
      (sched_get_priority_min(SCHED_FIFO) + sched_get_priority_max(SCHED_FIFO)) / 2 - 1;
  if (pthread_attr_init(&can_thread_attr))
  {
    RCUTILS_LOG_FATAL_NAMED("SocketCAN", "Can't init thread attribute");
    exit(-1);
  }
  // if (pthread_attr_setschedpolicy(&can_thread_attr, SCHED_FIFO))
  // {
  //   RCUTILS_LOG_FATAL_NAMED("SocketCAN", "Can't set thread policy");
  //   exit(-1);
  // }
  // if (pthread_attr_setinheritsched(&can_thread_attr, PTHREAD_EXPLICIT_SCHED))
  // {
  //   RCUTILS_LOG_FATAL_NAMED("SocketCAN", "Can't set thread inherit");
  //   exit(-1);
  // }
  // if (pthread_attr_setschedparam(&can_thread_attr, &can_thread_sched_param))
  // {
  //   RCUTILS_LOG_FATAL_NAMED("SocketCAN", "Can't set thread priority");
  //   exit(-1);
  // }
  if (int err = pthread_create(&rxThreadHandle_, &can_thread_attr, &SocketCan::rxThread, this))
  {
    RCUTILS_LOG_FATAL_NAMED("SocketCAN", "Can't create rx thread: %d", err);
    exit(errno);
  }
  if (int err = pthread_create(&txThreadHandle_, &can_thread_attr, &SocketCan::txThread, this))
  {
    RCUTILS_LOG_FATAL_NAMED("SocketCAN", "Can't create tx thread: %d", err);
    exit(errno);
  }
}

bool SocketCan::isOpen() const
{
  return sfd_ >= 0;
}

[[noreturn]] void* SocketCan::rxThread(void* argv)
{
  auto* self = reinterpret_cast<SocketCan*>(argv);
  struct timeval timeout
  {
  };
  fd_set descriptors;
  struct can_frame rx_frame
  {
  };
  for (;;)
  {
    timeout.tv_sec = 1;
    FD_ZERO(&descriptors);
    FD_SET(self->sfd_, &descriptors);
    if (!select(self->sfd_ + 1, &descriptors, nullptr, nullptr, &timeout))
    {
      continue;
    }
    if (!::read(self->sfd_, &rx_frame, sizeof(struct can_frame)))
    {
      continue;
    }
    if (!self->rxCallbacks_.empty())
    {
      for (auto& callback : self->rxCallbacks_)
      {
        if (callback(rx_frame, self))
        {
          break;
        }
      }
    }
    else
    {
      std::lock_guard<std::mutex> lock(self->rxMutex_);
      self->rxQueue_.push(rx_frame);
    }
    pthread_testcancel();
  }
}

[[noreturn]] void* SocketCan::txThread(void* argv)
{
  auto* self = reinterpret_cast<SocketCan*>(argv);
  for (;;)
  {
    if (self->txQueue_.empty())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    else
    {
      if (self->txMutex_.try_lock())
      {
        struct can_frame tx_frame = self->txQueue_.front();
        self->txQueue_.pop();
        if (!::write(self->sfd_, &tx_frame, sizeof(struct can_frame)))
        {
          RCUTILS_LOG_ERROR_NAMED("SocketCAN", "Can't write to socket: %s", strerror(errno));
        }
        self->txMutex_.unlock();
      }
    }
    pthread_testcancel();
  }
}

void SocketCan::bindRxCallback(const std::function<bool(struct can_frame&, SocketCan*)>& callback)
{
  std::lock_guard<std::mutex> lock(rxMutex_);
  rxCallbacks_.push_back(callback);
  rxQueue_ = std::queue<struct can_frame>();
}

void SocketCan::clearRxCallback()
{
  std::lock_guard<std::mutex> lock(rxMutex_);
  rxCallbacks_.clear();
  rxQueue_ = std::queue<struct can_frame>();
}

void SocketCan::close()
{
  if (isOpen())
  {
    pthread_cancel(rxThreadHandle_);
    pthread_join(rxThreadHandle_, nullptr);
    pthread_cancel(txThreadHandle_);
    pthread_join(txThreadHandle_, nullptr);
    ::close(sfd_);
    sfd_ = -1;
  }
  (void)system(("sudo ifconfig " + interfaceName_ + " down").c_str());
}

const SocketCan* SocketCan::operator<<(struct can_frame& frame)
{
  if (!isOpen())
  {
    RCUTILS_LOG_WARN_NAMED("SocketCAN", "Can't send frame, socket is not open");
  }
  std::lock_guard<std::mutex> lock(txMutex_);
  txQueue_.push(frame);
  return this;
}
const SocketCan* SocketCan::operator>>(struct can_frame& frame)
{
  if (!isOpen())
  {
    RCUTILS_LOG_WARN_NAMED("SocketCAN", "Can't receive frame, socket is not open");
  }
  if (!rxCallbacks_.empty())
  {
    RCUTILS_LOG_ERROR_NAMED("SocketCAN", "Can't receive frame, callback is set");
  }
  std::lock_guard<std::mutex> lock(rxMutex_);
  if (rxQueue_.empty())
  {
    memset(&frame, 0, sizeof(can_frame));
  }
  else
  {
    frame = rxQueue_.front();
    rxQueue_.pop();
  }
  return this;
}