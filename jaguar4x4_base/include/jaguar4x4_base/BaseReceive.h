// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <chrono>
#include <memory>
#include <utility>

#include <jaguar4x4_comms/AbstractCommunication.h>
#include <jaguar4x4_comms/MotorParse.h>

class AbstractBaseMsg {
 public:
  enum class MessageType {
    imu,
    gps,
    motor,
  };

  explicit AbstractBaseMsg(AbstractBaseMsg::MessageType msg_type)
    : msg_type_(msg_type)
  {
    std::chrono::time_point<std::chrono::system_clock,
      std::chrono::nanoseconds> now = std::chrono::system_clock::now();
    std::chrono::duration<uint64_t, std::nano> epoch = now.time_since_epoch();
    ts_s_ = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    ts_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - ts_s_);
  }

  virtual ~AbstractBaseMsg() = default;

  std::pair<std::chrono::seconds, std::chrono::nanoseconds> getTime() const
  {
    return std::make_pair(ts_s_, ts_ns_);
  }

  AbstractBaseMsg::MessageType getType() const
  {
    return msg_type_;
  }

protected:
  AbstractBaseMsg::MessageType msg_type_;
  std::chrono::seconds ts_s_;
  std::chrono::nanoseconds ts_ns_;
};

class ImuMsg final : public AbstractBaseMsg {
 public:
  explicit ImuMsg(uint8_t seqnum,
                  double yaw,
                  int gyro_x,
                  int gyro_y,
                  int gyro_z,
                  int accel_x,
                  int accel_y,
                  int accel_z,
                  int comp_x,
                  int comp_y,
                  int comp_z)
    : AbstractBaseMsg(AbstractBaseMsg::MessageType::imu),
      seqnum_(seqnum),
      yaw_(yaw),
      gyro_x_(gyro_x),
      gyro_y_(gyro_y),
      gyro_z_(gyro_z),
      accel_x_(accel_x),
      accel_y_(accel_y),
      accel_z_(accel_z),
      comp_x_(comp_x),
      comp_y_(comp_y),
      comp_z_(comp_z)
    {}

  uint8_t seqnum_;
  double yaw_;

  int gyro_x_;
  int gyro_y_;
  int gyro_z_;

  int accel_x_;
  int accel_y_;
  int accel_z_;

  int comp_x_;
  int comp_y_;
  int comp_z_;
};

class GPSMsg final : public AbstractBaseMsg {
 public:
  explicit GPSMsg(long timestamp,
                  long datestamp,
                  int status,
                  double lat,
                  double longitude,
                  double vog,
                  double cog)
    : AbstractBaseMsg(AbstractBaseMsg::MessageType::gps),
      timestamp_(timestamp),
      datestamp_(datestamp),
      status_(status),
      latitude_(lat),
      longitude_(longitude),
      vog_(vog),
      cog_(cog)
    {}

  long timestamp_;
  long datestamp_;
  int status_;
  double latitude_;
  double longitude_;
  double vog_;
  double cog_;
};

class MotorMsg final : public AbstractBaseMsg {
 public:
  explicit MotorMsg(uint8_t which, std::unique_ptr<AbstractMotorMsg> motor)
    : AbstractBaseMsg(AbstractBaseMsg::MessageType::motor),
    motor_num_(which),
    motor_msg_(std::move(motor))
    {}

  uint8_t motor_num_;
  std::unique_ptr<AbstractMotorMsg> motor_msg_;
};

class BaseReceive final {
 public:
  explicit BaseReceive(std::shared_ptr<AbstractCommunication> comm);
  std::unique_ptr<AbstractBaseMsg> getAndParseMessage();

private:
  std::shared_ptr<AbstractCommunication> comm_;
};
