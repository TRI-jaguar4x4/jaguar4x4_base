// Copyright 2018 Toyota Research Institute.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
