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

#include <iostream>
#include <memory>
#include <regex>
#include <string>

#include <jaguar4x4_comms/MotorParse.h>
#include <jaguar4x4_comms/Utils.h>

#include "jaguar4x4_base/BaseReceive.h"

BaseReceive::BaseReceive(std::shared_ptr<AbstractCommunication> comm) : comm_(comm)
{
}

static double transToDegree(double angle)
{
  double deg = 0;

  int degree = (int)(angle / 100);
  double min = ((angle - degree * 100));

  deg = degree + ((double)min) / 60;

  return deg;
}

static std::unique_ptr<AbstractBaseMsg> parseAndReturnIMUMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("#([0-9]*?),YAW,(-?[0-9]*\\.?[0-9]*?),GYRO,(-?[0-9]*?),(-?[0-9]*?),(-?[0-9]*?),ACC,(-?[0-9]*?),(-?[0-9]*?),(-?[0-9]*?),COMP,(-?[0-9]*?),(-?[0-9]*?),(-?[0-9]*?),ADC,(-?[0-9]*?),(-?[0-9]*?),(-?[0-9]*?),(-?[0-9]*?),$"))) {

    return std::make_unique<ImuMsg>(std::stol(sm[1]),  // seqnum
                                    str_to_d(sm[2]),   // yaw
                                    std::stol(sm[3]),  // gyro_x
                                    std::stol(sm[4]),  // gyro_y
                                    std::stol(sm[5]),  // gyro_z
                                    std::stol(sm[6]),  // accel_x
                                    std::stol(sm[7]),  // accel_y
                                    std::stol(sm[8]),  // accel_z
                                    std::stol(sm[9]),  // comp_x
                                    std::stol(sm[10]), // comp_y
                                    std::stol(sm[11])); // comp_z
  }

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractBaseMsg>(nullptr);
}

static std::unique_ptr<AbstractBaseMsg> parseAndReturnGPSMsg(const std::string& msg)
{
  std::smatch sm;

  static const double KNNOT2MS = 0.5144444;

  if (std::regex_match(msg, sm, std::regex("\\$GPRMC,(-?[0-9]*\\.?[0-9]*?),([AV]),(-?[0-9]*\\.?[0-9]*?),([NS]),(-?[0-9]*\\.?[0-9]*?),([EW]),(-?[0-9]*\\.?[0-9]*?),(-?[0-9]*\\.?[0-9]*?),([0-9]*?),(-?[0-9]*\\.?[0-9]*?),W.*$"))) {

    int datestamp = 0;

    int status = 0;
    if (sm[2] == "A") {
      status = 0;
    } else if (sm[2] == "V") {
      status = -1;
    } else {
      // this should never really happen; if it does, somehow we had an
      // invalid message *and* the regex above was wrong.  Log it, and
      // ignore the message.
      return std::unique_ptr<AbstractBaseMsg>(nullptr);
    }

    double lat = transToDegree(str_to_d(sm[3]));
    if (sm[4] == "N") {
      // Nothing to do
    } else if (sm[4] == "S") {
      lat *= -1;
    } else {
      // this should never really happen; if it does, somehow we had an
      // invalid message *and* the regex above was wrong.  Log it, and
      // ignore the message.
      return std::unique_ptr<AbstractBaseMsg>(nullptr);
    }

    double longitude = transToDegree(str_to_d(sm[5]));
    if (sm[6] == "E") {
      // Nothing to do
    } else if (sm[6] == "W") {
      longitude *= -1;
    } else {
      // this should never really happen; if it does, somehow we had an
      // invalid message *and* the regex above was wrong.  Log it, and
      // ignore the message.
      return std::unique_ptr<AbstractBaseMsg>(nullptr);
    }

    double vog = 0.0;
    if (status == 0) {
      vog = str_to_d(sm[7]) * KNNOT2MS;
    } else {
      if (sm[7] != "") {
        datestamp = std::stol(sm[7]);
      }
    }

    double cog = 0.0;
    if (status == 0) {
      cog = str_to_d(sm[8]);
    }

    return std::make_unique<GPSMsg>(std::stol(sm[1]),
                                    datestamp,
                                    status,
                                    lat,
                                    longitude,
                                    vog,
                                    cog);
  }

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractBaseMsg>(nullptr);
}

static std::unique_ptr<AbstractBaseMsg> parseAndReturnMotorMsg(const std::string& msg)
{
  std::smatch sm;

  if (std::regex_match(msg, sm, std::regex("MM([0-3]) (.*)"))) {
    std::unique_ptr<AbstractMotorMsg> motor_msg = parseMotorMessage(sm[2]);
    if (motor_msg) {
      return std::make_unique<MotorMsg>(std::stol(sm[1]), std::move(motor_msg));
    }
  }

  // If we got here, it failed to parse; just return empty
  return std::unique_ptr<AbstractBaseMsg>(nullptr);
}

std::unique_ptr<AbstractBaseMsg> BaseReceive::getAndParseMessage()
{
  std::string msg = comm_->recvMessage("\r\n", 100);

  // nothing before the "\r\n"
  if (msg.empty()) {
    return std::unique_ptr<AbstractBaseMsg>(nullptr);
  }

  if (startsWith(msg, "#")) {
    return parseAndReturnIMUMsg(msg);
  } else if (startsWith(msg, "$")) {
    return parseAndReturnGPSMsg(msg);
  } else if (startsWith(msg, "M")) {
    return parseAndReturnMotorMsg(msg);
  }

  return std::unique_ptr<AbstractBaseMsg>(nullptr);
}
