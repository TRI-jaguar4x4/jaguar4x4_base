// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <memory>
#include <mutex>

#include <jaguar4x4_comms/AbstractCommunication.h>

class BaseCommand final {
 public:
  explicit BaseCommand(std::shared_ptr<AbstractCommunication> comm);
  void move(int value_left, int value_right);
  void resume();
  void eStop();
  void ping();

private:
  std::shared_ptr<AbstractCommunication> comm_;
  std::mutex send_mutex;
};
