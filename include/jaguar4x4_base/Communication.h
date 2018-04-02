// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <string>

#include "AbstractCommunication.h"

class Communication final : public AbstractCommunication {
 public:
  void connect(const std::string& ip, uint16_t port) override;
  void sendCommand(const std::string& cmd) override;
  std::string recvMessage(const std::string& boundary, int timeout_msec)
    override;

 private:
  int ipValidator(const std::string& ip);
  int fd_;
  // TODO: restrict the partial_buffer so that it doesn't consume all of memory
  std::string partial_buffer_;
};
