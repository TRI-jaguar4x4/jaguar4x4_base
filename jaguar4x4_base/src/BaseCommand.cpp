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

#include <memory>
#include <mutex>
#include <string>

#include <jaguar4x4_comms/AbstractCommunication.h>

#include "jaguar4x4_base/BaseCommand.h"

BaseCommand::BaseCommand(std::shared_ptr<AbstractCommunication> comm)
  : comm_(comm)
{
}

void BaseCommand::move(int value_left, int value_right)
{
  std::string base_command("MMW !M ");
  base_command.append(std::to_string(value_left));
  base_command.append(" ");
  base_command.append(std::to_string(value_right));
  base_command.append("\r\n");

  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand(base_command);
}

void BaseCommand::resume()
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand("MMW !MG\r\n");
}

void BaseCommand::eStop()
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand("MMW !EX\r\n");
}

void BaseCommand::ping()
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand("PING\r\n");
  comm_->sendCommand("MMW ~MMOD\r\n");
}
