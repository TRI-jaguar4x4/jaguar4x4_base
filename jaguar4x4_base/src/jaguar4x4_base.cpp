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

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"

#include <jaguar4x4_comms/Communication.h>
#include "jaguar4x4_base/BaseCommand.h"
#include "jaguar4x4_base/BaseReceive.h"

#include "jaguar4x4_base_msgs/msg/motors.hpp" // pattern is all lower case name w/ underscores
#include "jaguar4x4_base_msgs/srv/base_rpm_calculator.hpp"
#include <jaguar4x4_comms_msgs/msg/motor_board.hpp> // pattern is all lower case name w/ underscores

using namespace std::chrono_literals;

class Jaguar4x4Base final : public rclcpp::Node
{
public:
  explicit Jaguar4x4Base(const std::string& ip, uint16_t port)
    : Node("jaguar4x4base")
  {
    pwm_to_speed_slope_ = (pwm_to_speed_start_point_.pwm_ - pwm_to_speed_end_point_.pwm_) / (pwm_to_speed_start_point_.speed_m_per_s_ - pwm_to_speed_end_point_.speed_m_per_s_);
    pwm_to_speed_y_intercept_ = pwm_to_speed_start_point_.pwm_ - (pwm_to_speed_slope_ * pwm_to_speed_start_point_.speed_m_per_s_);

    auto board_comm = std::make_shared<Communication>();
    board_comm->connect(ip, port);

    base_cmd_ = std::make_unique<BaseCommand>(board_comm);
    base_recv_ = std::make_unique<BaseReceive>(board_comm);

    future_ = exit_signal_.get_future();

    base_recv_thread_ = std::thread(&Jaguar4x4Base::baseRecvThread, this,
                                    future_);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu");

    navsat_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsat");

    motors_msg_ = std::make_unique<jaguar4x4_base_msgs::msg::Motors>();
    motors_pub_ = this->create_publisher<jaguar4x4_base_msgs::msg::Motors>("base_motors");
    motors_pub_thread_ = std::thread(&Jaguar4x4Base::motorsPubThread, this, future_);

    ping_thread_ = std::thread(&Jaguar4x4Base::pingThread, this, future_);

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

    rpm_calc_srv_ = this->create_service<jaguar4x4_base_msgs::srv::BaseRPMCalculator>("base_rpm_calculator",
                                                                                      std::bind(&Jaguar4x4Base::baseRPMCalculate, this, std::placeholders::_1, std::placeholders::_2));

    // TODO: The joystick callback runs on the same thread as the RPM
    // calculation service, so we can't currently eStop the RPM calculation.
    // We should figure out how to do that.
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",
                                                                std::bind(&Jaguar4x4Base::joyCallback, this, std::placeholders::_1),
                                                                cmd_vel_qos_profile);

    // sending this to make sure that the motors are not moving (they
    // seem to remember the last thing they were commanded to do)
    base_cmd_->move(0, 0);
    base_cmd_->eStop();
  }

  ~Jaguar4x4Base()
  {
    exit_signal_.set_value();
    base_recv_thread_.join();
    ping_thread_.join();
    motors_pub_thread_.join();
  }

private:
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
    // The ITG3205 datasheet says 14.375 deg/sec/LSB for 16g, which is
    // what we are configured for.
    imu_msg->angular_velocity.x = imu->gyro_x_ / 14.375; // all of the below are first approximation based on LSB
    imu_msg->angular_velocity.y = imu->gyro_y_ / 14.375;
    imu_msg->angular_velocity.z = imu->gyro_z_ / 14.375;
    // The ADXL345 datasheet says between 28.6 and 34.5, but empirically we
    // found we need to use 25.0 to get close to gravity.  Shrug.
    imu_msg->linear_acceleration.x = imu->accel_x_ / 25.0;
    imu_msg->linear_acceleration.y = imu->accel_y_ / 25.0;
    imu_msg->linear_acceleration.z = imu->accel_z_ / 25.0;
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
    int64_t old_encoder_diff1;
    int64_t old_encoder_diff2;

    if (motor->motor_num_ == 0) {
      motor_ref = &motors_msg_->motor1;
      old_encoder_diff1 = motors_msg_->motor1.encoder_diff_1;
      old_encoder_diff2 = motors_msg_->motor1.encoder_diff_2;
    } else if (motor->motor_num_ == 1) {
      motor_ref = &motors_msg_->motor2;
      old_encoder_diff1 = motors_msg_->motor2.encoder_diff_1;
      old_encoder_diff2 = motors_msg_->motor2.encoder_diff_2;
    } else {
      RCLCPP_WARN(get_logger(), "Invalid Motor %d", motor->motor_num_);
      return;
    }

    std::unique_ptr<AbstractMotorMsg>& msg = motor->motor_msg_;
    abstractMotorToROS(msg.get(), motor_ref);

    num_data_recvd_++;

    new_motor_data_ = true;

    // clalancette: When freewheeling, the base motors cannot start turning
    // until they get at least 35 to the PWM.
    if (msg->getType() == AbstractMotorMsg::MessageType::encoder_position_diff) {
      MotorEncPosDiffMsg *motor_enc_pos_diff = dynamic_cast<MotorEncPosDiffMsg*>(msg.get());

      // We accumulate encoder diffs here so that we have all of the encoder
      // changes since the last sensor frame.  All other values are instantaneous.
      motor_ref->encoder_diff_1 += old_encoder_diff1;
      motor_ref->encoder_diff_2 += old_encoder_diff2;
      if (enc_count_service_.running_ && motor->motor_num_ == enc_count_service_.motor_board_num_) {
        if (enc_count_service_.motor_num_ == 0) {
          enc_count_service_.counts_ += motor_enc_pos_diff->pos_diff_1_;
        } else {
          enc_count_service_.counts_ += motor_enc_pos_diff->pos_diff_2_;
        }
      }
    }
  }

  void baseRecvThread(std::shared_future<void> local_future)
  {
    std::future_status status;
    std::unique_ptr<AbstractBaseMsg> base_msg;

    do {
      try {
        base_msg = base_recv_->getAndParseMessage();
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

      base_msg.reset();

    } while (status == std::future_status::timeout);
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
    // PWM value | Speed (m/s)
    // ----------+-------------
    //     95    |    0.106
    //    100    |    0.124
    //    250    |    0.424
    //    500    |    0.884
    //    750    |    1.348 (calculated)
    //   1000    |    1.838 (calculated)
    //
    // (see the implementation of the baseRPMCalculate service for a discussion
    // of the procedure and math to determine this).
    //
    // Based on those values, we saw that it is basically a linear relationship,
    // so we can use classic slope/intercept formula of:
    //
    // y = m * x + b
    //
    // We calculated the slope based on rise/run:
    //
    // m = (500 - 95) / (0.884 - 0.106) = 520.70
    //
    // For a formula of:
    //
    // y = 520.70 * x + b
    //
    // Feeding one of our points (0.884, 500) back in, we can calculate b as:
    //
    // y - 520.70x = b
    // 500 - 520.70 * 0.884 = b
    // b = 39.70
    //
    // Putting that all together, we get a formula of:
    //
    // y = 520.70 * x + 39.70
    //
    // So we can feed a speed in m/s into x, and get a PWM value out.  There is
    // one more detail to note, which is that the left motor drives forward with
    // positive PWM, while the right motor drives backwards with positive PWM.
    // Thus, if we are driving the left motor, we *add* the y-intercept in the
    // above equation, and if we are driving the right motor, we *subtract* the
    // y-intercept in the above equation.

    double combined_vels;
    if (wheel == Wheel::LEFT_WHEEL) {
      combined_vels = linear_x - angular_z;
    } else {
      combined_vels = linear_x + angular_z;
    }

    double velocity = (combined_vels * JAGUAR_WHEEL_BASE_M / 2.0) / JAGUAR_WHEEL_RADIUS_M;

    double pwm = pwm_to_speed_slope_ * velocity;
    if (velocity < 0) {
      pwm -= pwm_to_speed_y_intercept_;
    } else if (velocity > 0) {
      pwm += pwm_to_speed_y_intercept_;
    }
    // implicit: if velocity == 0, don't touch PWM

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

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Button 11 is the Right Analog Stick press on the Logitech joystick, which
    // we use as "eStop the robot".  Button 10 is the Left Analog Stick press on
    // the joystick, which we use as "resume the robot".  We start out with the
    // robot in eStop, so you always must resume it to start using the robot.

    if (!msg->buttons[10] && !msg->buttons[11]) {
      return;
    }

    if (msg->buttons[11]) {
      // If we see eStop, set our eStopped_ atomic variable to true.  This will
      // ensure that the pingThread does not start accepting commands while we
      // are eStopped.
      eStopped_ = true;
      accepting_commands_ = false;

      // eStop the robot, and set the movement to 0.  The latter is so that the
      // robot won't continue moving at its last commanded power when we resume.
      base_cmd_->eStop();
      base_cmd_->move(0, 0);

      // We have to reset the prior PWM values here so that when we resume, we
      // actually start sending commands as expected.  Note that these are safe
      // to change without atomics because they are on the same thread as the
      // cmdVelCallback (the only other user of them).
      prior_pwm_left_ = 0.0;
      prior_pwm_right_ = 0.0;

      std::cerr << "ESTOP" << std::endl;
    } else {
      // Resume the robot.  We set eStopped to false, and then rely on the
      // pingThread to set accepting_commands to true as appropriate.
      base_cmd_->resume();
      eStopped_ = false;
    }
  }

  void pingThread(std::shared_future<void> local_future)
  {
    std::future_status status;

    std::chrono::time_point<std::chrono::system_clock> last_watchdog_check = std::chrono::system_clock::now();

    do {
      base_cmd_->ping();

      std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
      auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_watchdog_check);
      if (diff_ms.count() > kWatchdogIntervalMS) {
        if (!eStopped_) {
          if (num_data_recvd_ < kMinPingsExpected) {
            std::cerr << "Stopped accepting commands" << std::endl;
            accepting_commands_ = false;
          } else {
            if (!accepting_commands_) {
              std::cerr << "accepting commands" << std::endl;
              base_cmd_->move(0, 0);
            }
            accepting_commands_ = true;
          }
        }
        last_watchdog_check = now;
        num_data_recvd_ = 0;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(kPingTimerIntervalMS));

      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void motorsPubThread(std::shared_future<void> local_future)
  {
    std::future_status status;

    do {
      if (new_motor_data_) {
        // to set timestamp
        rcutils_time_point_value_t now;
        if (rcutils_system_time_now(&now) == RCUTILS_RET_OK) {
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

            motors_msg_->motor1.encoder_diff_1 = 0;
            motors_msg_->motor1.encoder_diff_2 = 0;

            motors_msg_->motor2.encoder_diff_1 = 0;
            motors_msg_->motor2.encoder_diff_2 = 0;

            new_motor_data_ = false;
          }

          motors_pub_->publish(motors_pub_msg);
        } else {
          std::cerr << "unable to access time\n";
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(kMotorsPubTimerIntervalMS));

      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  // This service allows an external observer to drive the PWM values of the
  // Jaguar4x4 motors and get an approximate, empirical value for the RPMs and
  // speed out.  The request parameters are:
  //
  // test_motor - Which motor to measure, one of FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, or REAR_RIGHT
  // test_motor_power - The PWM value to drive into the motor
  // test_duration_ms - How many milliseconds to run the test for
  //
  // Given the above parameters, this service will start driving the base at
  // the requested PWM values, measure the amount of time and number of encoder
  // ticks that elapse during the test, and then use the below formulas to
  // calculate the RPMs and speed observed.
  void baseRPMCalculate(const std::shared_ptr<jaguar4x4_base_msgs::srv::BaseRPMCalculator::Request> request,
                        std::shared_ptr<jaguar4x4_base_msgs::srv::BaseRPMCalculator::Response> response)
  {
    std::cerr << "Called calculate!" << std::endl;

    // Figure out which motor board and motor number we want to watch.
    if (request->test_motor == request->TEST_MOTOR_FRONT_LEFT) {
      enc_count_service_.motor_board_num_ = 0;
      enc_count_service_.motor_num_ = 0;
    } else if (request->test_motor == request->TEST_MOTOR_FRONT_RIGHT) {
      enc_count_service_.motor_board_num_ = 0;
      enc_count_service_.motor_num_ = 1;
    } else if (request->test_motor == request->TEST_MOTOR_REAR_LEFT) {
      enc_count_service_.motor_board_num_ = 1;
      enc_count_service_.motor_num_ = 0;
    } else if (request->test_motor == request->TEST_MOTOR_REAR_RIGHT) {
      enc_count_service_.motor_board_num_ = 1;
      enc_count_service_.motor_num_ = 1;
    } else {
      // error?  Return -1 for rpms
      response->rpms = -1.0;
      response->speed_m_per_s = -1.0;
      return;
    }

    // Reset our counters
    enc_count_service_.counts_ = 0;

    // Start the base moving
    auto start_time = std::chrono::high_resolution_clock::now();
    base_cmd_->move(request->test_motor_power, -request->test_motor_power);
    enc_count_service_.running_ = true;

    std::chrono::duration<double, std::milli> diff_ms;
    diff_ms = std::chrono::high_resolution_clock::now() - start_time;
    while (diff_ms.count() < request->test_duration_ms) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      diff_ms = std::chrono::high_resolution_clock::now() - start_time;
    }

    enc_count_service_.running_ = false;
    // Stop the base movement
    base_cmd_->move(0, 0);

    // Determining the RPMs from the number of encoder counts is a
    // straightforward affair.  We take the total number of counts observed,
    // then divide by the encoder counts per rotation (520.0) to determine
    // the number of full rotations observed.  We then divide that by the
    // number of milliseconds that elapsed to get revolutions per millisecond.
    // We then multiply that by 1000*60 to get the number of revolutions per
    // minute, for a final formula of:
    //
    // rpms = ((counts / 520.0) / time_elapsed_ms) * (1000.0 * 60)
    response->rpms = ((enc_count_service_.counts_ / JAGUAR_BASE_ENCODER_COUNTS_PER_REVOLUTION) / diff_ms.count()) * (1000.0 * 60.0);

    // Determining the speed from the RPMs is also straightforward, though we
    // need an additional piece of information.  Specifically, we need to know
    // how far the wheel goes per rotation, which we can get from the
    // circumference (we know the Jaguar4x4 wheel radius is 0.13 meters):
    //
    // circum_m = 2 * pi * r = 2 * pi * 0.13 = 0.817
    double circum_m = 2 * 3.14159 * JAGUAR_WHEEL_RADIUS_M;

    // Given that, we can now determine the speed in m/s by the following
    // formula:
    //
    // speed_m_per_s = rpms * circum_m / 60
    //
    // (rpms * circum_m determines the speed in meters/minute, then we divide
    // by 60 to get the speed in meters/second).
    response->speed_m_per_s = response->rpms * circum_m / 60.0;
  }

  struct Point final
  {
    double speed_m_per_s_;
    double pwm_;
  };

  // Tunable parameters
  const uint32_t kMotorsPubTimerIntervalMS = 100;
  const uint32_t kPingTimerIntervalMS = 40;
  const uint32_t kWatchdogIntervalMS = 200;
  static constexpr double kPingRecvPercentage = 0.8;
  static constexpr double JAGUAR_WHEEL_RADIUS_M = 0.1325;
  static constexpr double JAGUAR_WHEEL_BASE_M = 0.35;
  static constexpr double JAGUAR_BASE_ENCODER_COUNTS_PER_REVOLUTION = 520.0;
  const Point pwm_to_speed_start_point_{0.8840964279, 500};
  const Point pwm_to_speed_end_point_{0.1063000495, 95};

  // Constants calculated based on the tunable parameters above
  const uint32_t kPingsPerWatchdogInterval = kWatchdogIntervalMS / kPingTimerIntervalMS;
  const uint32_t kMinPingsExpected = kPingsPerWatchdogInterval * kPingRecvPercentage;

  double                                                                  pwm_to_speed_slope_;
  double                                                                  pwm_to_speed_y_intercept_;
  double                                                                  prior_pwm_left_{0.0};
  double                                                                  prior_pwm_right_{0.0};
  std::thread                                                             ping_thread_;
  std::thread                                                             motors_pub_thread_;
  std::thread                                                             joy_estop_thread_;
  std::atomic<uint32_t>                                                   num_data_recvd_{0};
  std::atomic<bool>                                                       accepting_commands_{false};
  std::unique_ptr<BaseReceive>                                            base_recv_;
  std::thread                                                             base_recv_thread_;
  std::unique_ptr<BaseCommand>                                            base_cmd_;
  std::promise<void>                                                      exit_signal_;
  std::shared_future<void>                                                future_;
  std::mutex                                                              motors_sensor_frame_mutex_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                     imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr               navsat_pub_;
  rclcpp::Publisher<jaguar4x4_base_msgs::msg::Motors>::SharedPtr          motors_pub_;  // base motors
  jaguar4x4_base_msgs::msg::Motors::UniquePtr                             motors_msg_;
  std::atomic<bool>                                                       new_motor_data_{false};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr              cmd_vel_sub_;
  rclcpp::Service<jaguar4x4_base_msgs::srv::BaseRPMCalculator>::SharedPtr rpm_calc_srv_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr                  joy_sub_;
  std::atomic<bool>                                                       eStopped_{true};
  struct EncoderCountService final
  {
    int counts_;
    std::atomic_bool running_;
    int motor_board_num_;
    int motor_num_;
  };
  EncoderCountService                                                     enc_count_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<Jaguar4x4Base>("192.168.0.60", 10001));
  rclcpp::shutdown();
  return 0;
}
