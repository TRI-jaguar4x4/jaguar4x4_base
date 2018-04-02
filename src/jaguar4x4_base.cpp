// Copyright 2018 Toyota Research Institute.  All rights reserved.
//
// IF WE RELEASE THIS CODE, WE MAY USE THE FOLLOWING BOILERPLATE:
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

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "jaguar4x4_base/Communication.h"
#include "jaguar4x4_base/BaseCommand.h"
#include "jaguar4x4_base/BaseReceive.h"

#include "jaguar4x4_base_msgs/msg/motors.hpp" // pattern is all lower case name w/ underscores
#include "jaguar4x4_base_msgs/msg/motor_board.hpp" // pattern is all lower case name w/ underscores

using namespace std::chrono_literals;

class Jaguar4x4Base final : public rclcpp::Node
{
public:
  explicit Jaguar4x4Base(const std::string& ip, uint16_t port)
  : Node("jaguar4x4")
  {
    auto board_comm = std::make_shared<Communication>();
    board_comm->connect(ip, port);

    base_cmd_ = std::make_unique<BaseCommand>(board_comm);
    base_recv_ = std::make_unique<BaseReceive>(board_comm);

    future_ = exit_signal_.get_future();

    base_recv_thread_ = std::thread(&Jaguar4x4Base::baseRecvThread, this,
                                    future_, std::move(base_recv_));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu");

    navsat_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsat");

    motors_pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(Jaguar4x4Base::kMotorsPubTimerIntervalMS),
      std::bind(&Jaguar4x4Base::motorsPubTimerCallback, this));
    motors_pub_ = this->create_publisher<jaguar4x4_base_msgs::msg::Motors>("base_motors");
    motors_msg_ = std::make_unique<jaguar4x4_base_msgs::msg::Motors>();

    // DDS has different quality of service protocols two basic ones...
    // best_effort and reliable (for reliability).
    // best_effort can publish to best_effort and reliable can publish to reliable
    // and a reliable can publish to a reliable, but a best effort cannot publish to a reliable
    // by default you get a reliable publisher, so to change that, specify a profile
    rmw_qos_profile_t cmd_vel_qos_profile = rmw_qos_profile_sensor_data;
    cmd_vel_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    cmd_vel_qos_profile.depth = 50;
    cmd_vel_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    cmd_vel_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
                                                                        std::bind(&Jaguar4x4Base::cmdVelCallback, this, std::placeholders::_1),
                                                                        cmd_vel_qos_profile);
  }

  ~Jaguar4x4Base()
  {
    exit_signal_.set_value();
    base_recv_thread_.join();
  }

private:
  const uint32_t kMotorsPubTimerIntervalMS = 100;

  void publishIMUMsg(AbstractBaseMsg* base_msg, rcutils_time_point_value_t& now)
  {
    ImuMsg* imu = dynamic_cast<ImuMsg*>(base_msg);
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

    imu_msg->header.stamp.sec = RCL_NS_TO_S(now);
    imu_msg->header.stamp.nanosec = now - RCL_S_TO_NS(imu_msg->header.stamp.sec);
    imu_msg->orientation.x = imu->comp_x_; // currently showing RAW magnetic sensor data
    imu_msg->orientation.y = imu->comp_y_; // currently showing RAW magnetic sensor data
    imu_msg->orientation.z = imu->comp_z_; // currently showing RAW magnetic sensor data
    imu_msg->orientation.w = 0.0;
    imu_msg->angular_velocity.x = imu->gyro_x_ / 14.375; // all of the below are first approximation based on LSB
    imu_msg->angular_velocity.y = imu->gyro_y_ / 14.375;
    imu_msg->angular_velocity.z = imu->gyro_z_ / 14.375;
    imu_msg->linear_acceleration.x = imu->accel_x_ / 32.0;
    imu_msg->linear_acceleration.y = imu->accel_y_ / 32.0;
    imu_msg->linear_acceleration.z = imu->accel_z_ / 32.0;
    // don't know covariances, they're defaulting to 0
    imu_pub_->publish(imu_msg);
  }

  void publishGPSMsg(AbstractBaseMsg* base_msg, rcutils_time_point_value_t& now)
  {
    GPSMsg* gps = dynamic_cast<GPSMsg*>(base_msg);
    auto navsat_fix_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();

    navsat_fix_msg->header.stamp.sec = RCL_NS_TO_S(now);
    navsat_fix_msg->header.stamp.nanosec = now - RCL_S_TO_NS(navsat_fix_msg->header.stamp.sec);
    switch (gps->status_) {
    case -1: // invalid
      navsat_fix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
    case 0:  // fixed
      navsat_fix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case 1:  // differential (assuming satellite based differential - vs ground based)
      navsat_fix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      break;
    default:
      RCLCPP_WARN(get_logger(), "Invalid GPS status %d", gps->status_);
      break;
    }
    navsat_fix_msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    navsat_fix_msg->latitude = gps->latitude_;
    navsat_fix_msg->longitude = gps->longitude_;
    navsat_fix_msg->altitude = 0.0;
    navsat_fix_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    navsat_pub_->publish(navsat_fix_msg);
  }

  void updateMotorMsg(AbstractBaseMsg* base_msg)
  {
    std::lock_guard<std::mutex> sf_lock_guard(motors_sensor_frame_mutex_);
    MotorMsg* motor = dynamic_cast<MotorMsg*>(base_msg);
    jaguar4x4_base_msgs::msg::MotorBoard* motor_ref;

    if (motor->motor_num_ == 0) {
      motor_ref = &motors_msg_->motor1;
    } else if (motor->motor_num_ == 1) {
      motor_ref = &motors_msg_->motor2;
    } else {
      RCLCPP_WARN(get_logger(), "Invalid Motor %d", motor->motor_num_);
      return;
    }

    std::unique_ptr<AbstractMotorMsg>& msg = motor->motor_msg_;
    switch(msg->getType()) {
    case AbstractMotorMsg::MessageType::motor_amperage:
      {
        MotorAmpMsg *motor_amp = dynamic_cast<MotorAmpMsg*>(msg.get());
        motor_ref->amp_1 = motor_amp->motor_amp_1_;
        motor_ref->amp_2 = motor_amp->motor_amp_2_;
      }
      break;
    case AbstractMotorMsg::MessageType::motor_temperature:
      {
        MotorTempMsg *motor_temp = dynamic_cast<MotorTempMsg*>(msg.get());
        motor_ref->motor_temp_1 = motor_temp->motor_temp_1_;
        motor_ref->motor_temp_2 = motor_temp->motor_temp_2_;
      }
      break;
    case AbstractMotorMsg::MessageType::encoder_position:
      {
        MotorEncPosMsg *motor_enc_pos =
          dynamic_cast<MotorEncPosMsg*>(msg.get());
        motor_ref->encoder_pos_1 = motor_enc_pos->encoder_pos_1_;
        motor_ref->encoder_pos_2 = motor_enc_pos->encoder_pos_2_;
      }
      break;
    case AbstractMotorMsg::MessageType::encoder_position_diff:
      {
        MotorEncPosDiffMsg *motor_enc_pos_diff =
          dynamic_cast<MotorEncPosDiffMsg*>(msg.get());
        motor_ref->encoder_diff_1 = motor_enc_pos_diff->pos_diff_1_;
        motor_ref->encoder_diff_2 = motor_enc_pos_diff->pos_diff_2_;
      }
      break;
    case AbstractMotorMsg::MessageType::digital_input:
      {
        MotorDigitalInputMsg *motor_digital_input =
          dynamic_cast<MotorDigitalInputMsg*>(msg.get());
        motor_ref->digital_input = motor_digital_input->digital_input_;
      }
      break;
    case AbstractMotorMsg::MessageType::digital_output:
      {
        MotorDigitalOutputMsg *motor_digital_output =
          dynamic_cast<MotorDigitalOutputMsg*>(msg.get());
        motor_ref->digital_output = motor_digital_output->digital_output_;
      }
      break;
    case AbstractMotorMsg::MessageType::motor_power:
      {
        // TODO we've seen this coming out negative... what's up with that?
        // TODO what are the units?  What does this number mean?
        // 25 for power 1 and 0 for power 2 on the arm,
        // -22 and -1 for power on the arm?
        MotorPowerMsg *motor_power =
          dynamic_cast<MotorPowerMsg*>(msg.get());
        motor_ref->motor_power_1 = motor_power->motor_power_1_;
        motor_ref->motor_power_2 = motor_power->motor_power_2_;
      }
      break;
    case AbstractMotorMsg::MessageType::encoder_velocity:
      {
        MotorEncVelMsg *motor_enc_vel =
          dynamic_cast<MotorEncVelMsg*>(msg.get());
        motor_ref->encoder_vel_1 = motor_enc_vel->encoder_velocity_1_;
        motor_ref->encoder_vel_2 = motor_enc_vel->encoder_velocity_2_;
      }
      break;
    case AbstractMotorMsg::MessageType::board_temperature:
      {
        MotorBoardTempMsg *motor_board_temp =
          dynamic_cast<MotorBoardTempMsg*>(msg.get());
        motor_ref->board_temp_1 = motor_board_temp->board_temp_1_;
        motor_ref->board_temp_2 = motor_board_temp->board_temp_2_;
      }
      break;
    case AbstractMotorMsg::MessageType::voltage:
      {
        MotorVoltageMsg *motor_voltage =
          dynamic_cast<MotorVoltageMsg*>(msg.get());
        motor_ref->volt_main = motor_voltage->bat_voltage_;
        motor_ref->volt_12v = motor_voltage->drv_voltage_;
        motor_ref->volt_5v = motor_voltage->reg_5_voltage_;
      }
      break;
    case AbstractMotorMsg::MessageType::motor_mode:
      {
        MotorModeMsg *motor_mode = dynamic_cast<MotorModeMsg*>(msg.get());
        motor_ref->motor_control_mode_1 =
          static_cast<std::underlying_type
                      <MotorModeMsg::MotorControlMode>::type>
          (motor_mode->mode_channel_1_);
        motor_ref->motor_control_mode_2 =
          static_cast<std::underlying_type
                      <MotorModeMsg::MotorControlMode>::type>
          (motor_mode->mode_channel_2_);
      }
      break;
    case AbstractMotorMsg::MessageType::motor_flags:
      {
        MotorFlagsMsg *motor_flags =
          dynamic_cast<MotorFlagsMsg*>(msg.get());
        motor_ref->overheat = motor_flags->overheat_;
        motor_ref->overvoltage = motor_flags->overvoltage_;
        motor_ref->undervoltage = motor_flags->undervoltage_;
        motor_ref->e_short = motor_flags->short_;
        motor_ref->estop = motor_flags->ESTOP_;
      }
      break;
    case AbstractMotorMsg::MessageType::command_accepted:
      {
        motor_ref->last_cmd_ack = true;
      }
      break;
    case AbstractMotorMsg::MessageType::command_rejected:
      {
        motor_ref->last_cmd_ack = false;
      }
      break;
    }
  }

  void baseRecvThread(std::shared_future<void> local_future,
                      std::unique_ptr<BaseReceive> base_recv)
  {
    std::future_status status;
    std::unique_ptr<AbstractBaseMsg> base_msg;

    do {
      try {
        base_msg = base_recv->getAndParseMessage();
      } catch (...) {
        std::cerr << "threw\n";
      }

      rcutils_time_point_value_t now;
      if (base_msg && rcutils_system_time_now(&now) == RCUTILS_RET_OK) {
        switch(base_msg->getType()) {
        case AbstractBaseMsg::MessageType::imu:
          publishIMUMsg(base_msg.get(), now);
          break;
        case AbstractBaseMsg::MessageType::gps:
          publishGPSMsg(base_msg.get(), now);
          break;
        case AbstractBaseMsg::MessageType::motor:
          updateMotorMsg(base_msg.get());
          break;
        }
      }

      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void copyMotorBoardMessage(jaguar4x4_base_msgs::msg::MotorBoard* to, jaguar4x4_base_msgs::msg::MotorBoard* from)
  {
    to->amp_1 = from->amp_1;
    to->amp_2 = from->amp_2;
    to->motor_temp_1 = from->motor_temp_1;
    to->motor_temp_2 = from->motor_temp_2;
    to->encoder_pos_1 = from->encoder_pos_1;
    to->encoder_pos_2 = from->encoder_pos_2;
    to->encoder_diff_1 = from->encoder_diff_1;
    to->encoder_diff_2 = from->encoder_diff_2;
    to->digital_input = from->digital_input;
    to->digital_output = from->digital_output;
    to->motor_power_1 = from->motor_power_1;
    to->motor_power_2 = from->motor_power_2;
    to->encoder_vel_1 = from->encoder_vel_1;
    to->encoder_vel_2 = from->encoder_vel_2;
    to->board_temp_1 = from->board_temp_1;
    to->board_temp_2 = from->board_temp_2;
    to->volt_main = from->volt_main;
    to->volt_12v = from->volt_12v;
    to->volt_5v = from->volt_5v;
    to->motor_control_mode_1 = from->motor_control_mode_1;
    to->motor_control_mode_2 = from->motor_control_mode_2;
    to->overheat = from->overheat;
    to->overvoltage = from->overvoltage;
    to->undervoltage = from->undervoltage;
    to->e_short = from->e_short;
    to->estop = from->estop;
    to->last_cmd_ack = from->last_cmd_ack;
  }

  void motorsPubTimerCallback()
  {
    auto motors_pub_msg = std::make_unique<jaguar4x4_base_msgs::msg::Motors>();

    // to set timestamp
    rcutils_time_point_value_t now;
    if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
      std::cerr << "unable to access time\n";
      return;
    }

    motors_pub_msg->header.stamp.sec = RCL_NS_TO_S(now);
    motors_pub_msg->header.stamp.nanosec =
      now - RCL_S_TO_NS(motors_pub_msg->header.stamp.sec);

    std::lock_guard<std::mutex> sf_lock_guard(motors_sensor_frame_mutex_);
    // Both motors_pub_msg->motor* and motors_msg_->motor* are of type
    // jaguar4x4_base_msgs::msg::MotorBoard, which is a class generated by the
    // ROS2 generator.  Those classes do not have copy constructors or =operator
    // implemented, so we do it by hand here.
    copyMotorBoardMessage(&(motors_pub_msg->motor1), &(motors_msg_->motor1));
    copyMotorBoardMessage(&(motors_pub_msg->motor2), &(motors_msg_->motor2));

    motors_pub_->publish(motors_pub_msg);
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Time to operate the robot!");

    float linearX = msg->linear.x * 350;
    float angularZ = msg->angular.z * 350;

    if (linearX != 0.0) {
      base_cmd_->resume();
      base_cmd_->move(linearX, -linearX);
    }

    if (angularZ != 0.0) {
      base_cmd_->resume();
      base_cmd_->move(angularZ, angularZ);
    }

    if (linearX == 0.0 && angularZ == 0.0) { // send stop
      base_cmd_->eStop();
    }
  }

  std::unique_ptr<BaseReceive>                                   base_recv_;
  std::thread                                                    base_recv_thread_;
  std::unique_ptr<BaseCommand>                                   base_cmd_;
  std::promise<void>                                             exit_signal_;
  std::shared_future<void>                                       future_;
  std::mutex                                                     motors_sensor_frame_mutex_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr            imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr      navsat_pub_;
  rclcpp::Publisher<jaguar4x4_base_msgs::msg::Motors>::SharedPtr motors_pub_;  // base motors
  jaguar4x4_base_msgs::msg::Motors::UniquePtr                    motors_msg_;
  rclcpp::TimerBase::SharedPtr                                   motors_pub_timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<Jaguar4x4Base>("192.168.0.60", 10001));
  rclcpp::shutdown();
  return 0;
}
