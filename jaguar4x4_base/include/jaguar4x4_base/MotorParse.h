// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <chrono>
#include <utility>

#include "AbstractCommunication.h"

class AbstractMotorMsg {
 public:
  enum class MessageType {
    motor_amperage,        // A
    motor_temperature,     // AI
    encoder_position,      // C
    encoder_position_diff, // CR
    digital_input,         // D
    digital_output,        // DO
    motor_power,           // P
    encoder_velocity,      // S
    board_temperature,     // T
    voltage,               // V
    motor_mode,            // MMOD
    motor_flags,           // FF
    command_accepted,      // +
    command_rejected,      // -
  };

  explicit AbstractMotorMsg(AbstractMotorMsg::MessageType msg_type)
    : msg_type_(msg_type) {
    std::chrono::time_point<std::chrono::system_clock,
                            std::chrono::nanoseconds> t;
    std::chrono::duration<int, std::nano> now = t.time_since_epoch();
    ts_s_ = std::chrono::duration_cast<std::chrono::seconds>(now);
    ts_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  }

  virtual ~AbstractMotorMsg() = default;

  std::pair<std::chrono::seconds, std::chrono::nanoseconds> getTime() const
  {
    return std::make_pair(ts_s_, ts_ns_);
  }

  AbstractMotorMsg::MessageType getType() const
  {
    return msg_type_;
  }

protected:
  AbstractMotorMsg::MessageType msg_type_;
  std::chrono::seconds ts_s_;
  std::chrono::nanoseconds ts_ns_;
};

class MotorAmpMsg final : public AbstractMotorMsg {
 public:
  explicit MotorAmpMsg(double amp1, double amp2)
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::motor_amperage),
      motor_amp_1_(amp1),
      motor_amp_2_(amp2)
  {}

  double motor_amp_1_;
  double motor_amp_2_;
};

class MotorTempMsg final : public AbstractMotorMsg {
 public:
  explicit MotorTempMsg(uint16_t temp1, uint16_t temp2);

  uint16_t motor_temp_adc_1_;
  double motor_temp_1_;
  uint16_t motor_temp_adc_2_;
  double motor_temp_2_;
};

class MotorEncPosMsg final : public AbstractMotorMsg {
 public:
  explicit MotorEncPosMsg(int64_t enc_pos_1, int64_t enc_pos_2)
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::encoder_position),
      encoder_pos_1_(enc_pos_1),
      encoder_pos_2_(enc_pos_2)
  {}

  int64_t encoder_pos_1_;
  int64_t encoder_pos_2_;
};

class MotorPowerMsg final : public AbstractMotorMsg {
 public:
  explicit MotorPowerMsg(int16_t motor_power_1, int16_t motor_power_2)
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::motor_power),
      motor_power_1_(motor_power_1),
      motor_power_2_(motor_power_2)
  {}

  int16_t motor_power_1_;
  int16_t motor_power_2_;
};

class MotorEncVelMsg final : public AbstractMotorMsg {
 public:
  explicit MotorEncVelMsg(int64_t encoder_velocity_1, int64_t encoder_velocity_2)
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::encoder_velocity),
      encoder_velocity_1_(encoder_velocity_1),
      encoder_velocity_2_(encoder_velocity_2)
  {}

  int64_t encoder_velocity_1_;
  int64_t encoder_velocity_2_;
};

class MotorBoardTempMsg final : public AbstractMotorMsg {
 public:
  explicit MotorBoardTempMsg(double temp_1, double temp_2)
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::board_temperature),
      board_temp_1_(temp_1),
      board_temp_2_(temp_2)
  {}

  double board_temp_1_;
  double board_temp_2_;
};

class MotorVoltageMsg final : public AbstractMotorMsg {
 public:
  explicit MotorVoltageMsg(double v_1, double v_2, double v_3)
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::voltage),
      drv_voltage_(v_1),
      bat_voltage_(v_2),
      reg_5_voltage_(v_3)
  {}

  double drv_voltage_;
  double bat_voltage_;
  double reg_5_voltage_;
};

class MotorFlagsMsg final : public AbstractMotorMsg {
 public:
  explicit MotorFlagsMsg(uint16_t flag) :
    AbstractMotorMsg(AbstractMotorMsg::MessageType::motor_flags)
  {
    overheat_ = (flag & 0x01);
    overvoltage_ = (flag & 0x02);
    undervoltage_ = (flag & 0x04);
    short_ = (flag & 0x08);
    ESTOP_ = (flag & 0x10);
  }

  bool overheat_;
  bool overvoltage_;
  bool undervoltage_;
  bool short_;
  bool ESTOP_;
};

class MotorModeMsg final : public AbstractMotorMsg {
 public:
  enum class MotorControlMode {
    OpenLoop = 0,
    ClosedSpeed = 1,
    Position_2 = 2,
    Position_3 = 3,
    Torque = 4,
  };

  explicit MotorModeMsg(int mode_1, int mode_2)
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::motor_mode),
      mode_channel_1_(MotorModeMsg::MotorControlMode(mode_1)),
      mode_channel_2_(MotorModeMsg::MotorControlMode(mode_2))
  {}

  MotorModeMsg::MotorControlMode mode_channel_1_;
  MotorModeMsg::MotorControlMode mode_channel_2_;
};

class MotorCmdAcceptedMsg final : public AbstractMotorMsg {
 public:
  MotorCmdAcceptedMsg()
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::command_accepted)
  {}
};

class MotorCmdRejectedMsg final : public AbstractMotorMsg {
 public:
  MotorCmdRejectedMsg()
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::command_rejected)
  {}
};

class MotorDigitalInputMsg final : public AbstractMotorMsg {
 public:
  explicit MotorDigitalInputMsg(int input)
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::digital_input),
      digital_input_(input)
  {}

  int digital_input_;
};

class MotorDigitalOutputMsg final : public AbstractMotorMsg {
 public:
  explicit MotorDigitalOutputMsg(int output)
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::digital_output),
      digital_output_(output)
  {}

  int digital_output_;
};

class MotorEncPosDiffMsg final : public AbstractMotorMsg {
 public:
  explicit MotorEncPosDiffMsg(int pos_diff_1, int pos_diff_2)
    : AbstractMotorMsg(AbstractMotorMsg::MessageType::encoder_position_diff),
      pos_diff_1_(pos_diff_1),
      pos_diff_2_(pos_diff_2)
  {}

  int pos_diff_1_;
  int pos_diff_2_;
};

std::unique_ptr<AbstractMotorMsg> parseMotorMessage(const std::string& msg);
