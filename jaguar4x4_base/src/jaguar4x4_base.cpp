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

#include <atomic>
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

#include <jaguar4x4_comms/Communication.h>
#include "jaguar4x4_base/BaseCommand.h"
#include "jaguar4x4_base/BaseReceive.h"

#include "jaguar4x4_base_msgs/msg/motors.hpp" // pattern is all lower case name w/ underscores
#include <jaguar4x4_comms_msgs/msg/motor_board.hpp> // pattern is all lower case name w/ underscores

using namespace std::chrono_literals;

class Jaguar4x4Base final : public rclcpp::Node
{
public:
  explicit Jaguar4x4Base(const std::string& ip, uint16_t port)
    : Node("jaguar4x4"),
      num_pings_sent_(0), num_pings_recvd_(0), accepting_commands_(false)
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

    ping_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(Jaguar4x4Base::kPingTimerIntervalMS),
      std::bind(&Jaguar4x4Base::pingTimerCallback, this));

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
    // sending this to make sure that the motors are not moving (they
    // seem to remember the last thing they were commanded to do)
    base_cmd_->move(0, 0);
  }

  ~Jaguar4x4Base()
  {
    exit_signal_.set_value();
    base_recv_thread_.join();
  }

private:
  const uint32_t kMotorsPubTimerIntervalMS = 100;

  // TODO: find out how long it takes the robot to respond.
  // Bumping kPingRecvPercentage_ down to .6 from .8 seems to make things work
  const uint32_t kPingTimerIntervalMS = 50;
  const uint32_t kWatchdogIntervalMS = 500;
  static constexpr double   kPingRecvPercentage = 0.6;
  const uint32_t kPingsPerWatchdogInterval = kWatchdogIntervalMS/kPingTimerIntervalMS;
  const uint32_t kMinPingsExpected = kPingsPerWatchdogInterval*kPingRecvPercentage;

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
    jaguar4x4_comms_msgs::msg::MotorBoard* motor_ref;

    if (motor->motor_num_ == 0) {
      motor_ref = &motors_msg_->motor1;
    } else if (motor->motor_num_ == 1) {
      motor_ref = &motors_msg_->motor2;
    } else {
      RCLCPP_WARN(get_logger(), "Invalid Motor %d", motor->motor_num_);
      return;
    }

    std::unique_ptr<AbstractMotorMsg>& msg = motor->motor_msg_;
    abstractMotorToROS(msg.get(), motor_ref);

    if (msg->getType() == AbstractMotorMsg::MessageType::motor_mode) {
      num_pings_recvd_++;
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

  void motorsPubTimerCallback()
  {
    // to set timestamp
    rcutils_time_point_value_t now;
    if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
      std::cerr << "unable to access time\n";
      return;
    }

    auto motors_pub_msg = std::make_unique<jaguar4x4_base_msgs::msg::Motors>();

    motors_pub_msg->header.stamp.sec = RCL_NS_TO_S(now);
    motors_pub_msg->header.stamp.nanosec =
      now - RCL_S_TO_NS(motors_pub_msg->header.stamp.sec);

    {
      std::lock_guard<std::mutex> sf_lock_guard(motors_sensor_frame_mutex_);
      // Both motors_pub_msg->motor* and motors_msg_->motor* are of type
      // jaguar4x4_comms_msgs::msg::MotorBoard, which is a class generated by the
      // ROS2 generator.  Those classes do not have copy constructors or =operator
      // implemented, so we copy by hand here.
      copyMotorBoardMessage(&(motors_pub_msg->motor1), &(motors_msg_->motor1));
      copyMotorBoardMessage(&(motors_pub_msg->motor2), &(motors_msg_->motor2));
    }

    motors_pub_->publish(motors_pub_msg);
  }

  enum class Wheel
  {
    LEFT_WHEEL,
    RIGHT_WHEEL,
  };

  double calcPWMFromTwistVels(double linear_x, double angular_z, Wheel wheel)
  {
    // In order to go from a Twist to the Differential Drive, we are using the
    // equations from http://robotsforroboticists.com/drive-kinematics/ .  In
    // particular:
    //
    // velocity_left_cmd = ((linear_velocity – angular_velocity) * WHEEL_BASE / 2.0)/WHEEL_RADIUS;
    // velocity_right_cmd = ((linear_velocity + angular_velocity) * WHEEL_BASE / 2.0)/WHEEL_RADIUS;
    //
    // (note that the original fails to include the parentheses around the
    // velocities, but that is necessary).
    //
    // From measurement, the Jaguar4x4 robot has a wheel radius of 0.13 meters
    // and a Wheel Base of 0.335 meters.
    //
    // However, that is not enough for us to actually drive the robot.  We have
    // to provide the Jaguar4x4 with Motor power (PWM values) to actually drive
    // the wheels.  Thus, we have to take the velocities that we calculated
    // above and transform them into PWM.
    //
    // Based on empirical evidence, we found the following approximate table
    // of values:
    //
    // PWM value |   RPMs
    // ----------+----------
    //     95    |    7.81
    //    100    |    9.09
    //    250    |   31.14
    //    500    |   64.94
    //    750    |   99.00 (calculated)
    //   1000    |  135.00 (calculated)
    //
    // Based on those values, we saw that it is basically a linear relationship.
    // We calculated the slope based on (1000 - 95) / (135 - 7.81) = 522.66.
    // Feeding that back into y = 522.66x + b, with a point of (135, 1000), we
    // calculated b as 39.44 .  Thus, feeding a speed of x in m/s in, we can get
    // the approximate PWM value out.

    static const double JAGUAR_WHEEL_BASE_M = 0.335;
    static const double JAGUAR_WHEEL_RADIUS_M = 0.13;

    double combined_vels;
    if (wheel == Wheel::LEFT_WHEEL) {
      combined_vels = linear_x - angular_z;
    } else {
      combined_vels = linear_x + angular_z;
    }

    double velocity = (combined_vels * JAGUAR_WHEEL_BASE_M / 2.0) / JAGUAR_WHEEL_RADIUS_M;

    double pwm = 522.65 * velocity;
    if (velocity == 0) {
      pwm = 0.0;
    } else if (velocity < 0) {
      pwm -= 39.44;
    } else {
      pwm += 39.44;
    }

    return pwm;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Time to operate the robot!");

    if (!accepting_commands_) {
      return;
    }

    double pwm_left = calcPWMFromTwistVels(msg->linear.x, msg->angular.z, Wheel::LEFT_WHEEL);
    double pwm_right = calcPWMFromTwistVels(msg->linear.x, msg->angular.z, Wheel::RIGHT_WHEEL);

    std::stringstream ss;

    ss << "Linear: " << msg->linear.x << ", angular: " << msg->angular.z << ", pwm_left: " << pwm_left << ", pwm_right: " << pwm_right;
    RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());

    if (prior_pwm_left_ != pwm_left || prior_pwm_right_ != pwm_right) {
      base_cmd_->move(-pwm_right, pwm_left);
      prior_pwm_left_ = pwm_left;
      prior_pwm_right_ = pwm_right;
    }
  }

  void pingTimerCallback()
  {
    // NOTE:  this implementation causes accepting_commands_ to be false
    // for the first 500ms of operation
    base_cmd_->ping();
    num_pings_sent_++;

    if (num_pings_sent_ < kPingsPerWatchdogInterval) {
      return;
    }

    if (num_pings_recvd_ < kMinPingsExpected) {
      accepting_commands_ = false;
    } else {
      if (!accepting_commands_) {
        std::cerr << "accepting commands\n";
        base_cmd_->move(0,0);
        base_cmd_->resume();
      }
      accepting_commands_ = true;
    }
    num_pings_recvd_ = 0;
    num_pings_sent_ = 0;
  }

  double                                                         prior_pwm_left_{0.0};
  double                                                         prior_pwm_right_{0.0};
  rclcpp::TimerBase::SharedPtr                                   ping_timer_;
  uint32_t                                                       num_pings_sent_;
  std::atomic<uint32_t>                                          num_pings_recvd_;
  std::atomic<bool>                                              accepting_commands_;
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
