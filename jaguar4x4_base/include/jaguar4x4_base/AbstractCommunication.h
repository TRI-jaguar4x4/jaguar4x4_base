// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <string>

class AbstractCommunication {
  // TODO: how does TRI feel about exceptions?
  // Are they allowed... if not, return code instead of void.
  // we probably want a disconnect... or do we want to connect
  // in the constructor, disconnect in the destructor?
 public:
  virtual void connect(const std::string& ip, uint16_t port) = 0;
  virtual void sendCommand(const std::string& cmd) = 0;
  virtual std::string recvMessage(const std::string& boundary,
                                  int timeout_msec) = 0;
};
