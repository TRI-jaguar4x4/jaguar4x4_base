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
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <jaguar4x4_comms/Communication.h>
#include "jaguar4x4_base/BaseCommand.h"
#include "jaguar4x4_base/BaseReceive.h"

#include "jaguar4x4_base_msgs/msg/motors.hpp" // pattern is all lower case name w/ underscores
#include "jaguar4x4_base_msgs/srv/base_rpm_calculator.hpp"
#include <jaguar4x4_comms_msgs/msg/motor_board.hpp> // pattern is all lower case name w/ underscores

#include <ecl/mobile_robot.hpp>
#include <ecl/linear_algebra.hpp>

using namespace std::chrono_literals;

class Jaguar4x4Base final : public rclcpp::Node
{
public:
  explicit Jaguar4x4Base(const std::string& ip, uint16_t port)
    : Node("jaguar4x4base")
  {
    pwm_to_speed_slope_ = (PWM_TO_SPEED_START_POINT_.pwm_ - PWM_TO_SPEED_END_POINT_.pwm_) / (PWM_TO_SPEED_START_POINT_.speed_m_per_s_ - PWM_TO_SPEED_END_POINT_.speed_m_per_s_);
    pwm_to_speed_y_intercept_ = PWM_TO_SPEED_START_POINT_.pwm_ - (pwm_to_speed_slope_ * PWM_TO_SPEED_START_POINT_.speed_m_per_s_);

    auto board_comm = std::make_shared<Communication>();
    board_comm->connect(ip, port);

    base_cmd_ = std::make_unique<BaseCommand>(board_comm);
    base_recv_ = std::make_unique<BaseReceive>(board_comm);

    future_ = exit_signal_.get_future();

    base_recv_thread_ = std::thread(&Jaguar4x4Base::baseRecvThread, this,
                                    future_);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu",
                                                             rmw_qos_profile_sensor_data);

    navsat_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsat",
                                                                      rmw_qos_profile_sensor_data);

    rmw_qos_profile_t tf_qos_profile = rmw_qos_profile_default;
    tf_qos_profile.depth = 100;
    tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", tf_qos_profile);

    motors_msg_ = std::make_unique<jaguar4x4_base_msgs::msg::Motors>();
    motors_pub_ = this->create_publisher<jaguar4x4_base_msgs::msg::Motors>("base_motors",
                                                                           rmw_qos_profile_sensor_data);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom",
                                                                rmw_qos_profile_sensor_data);
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

    // We use a separate callback group for the joystick callback so that a
    // thread can service this separate from all of the other callbacks,
    // including services.
    joy_cb_grp_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",
                                                                std::bind(&Jaguar4x4Base::joyCallback, this, std::placeholders::_1),
                                                                cmd_vel_qos_profile,
                                                                joy_cb_grp_);

    travel_one_meter_srv_ = this->create_service<std_srvs::srv::Trigger>("travel_one_meter",
                                                                         std::bind(&Jaguar4x4Base::travel_one_meter,  this, std::placeholders::_1, std::placeholders::_2));

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
  inline double deg_per_sec_to_rad_per_sec(double deg_per_sec)
  {
    return deg_per_sec * JAGUAR_PI / 180.0;
  }

  void collectGyroData(ImuMsg* imu)
  {
    gyro_bias_calc_.sum_x_ += imu->gyro_x_;
    gyro_bias_calc_.sum_y_ += imu->gyro_y_;
    gyro_bias_calc_.sum_z_ += imu->gyro_z_;
    gyro_bias_calc_.num_samples_ += 1;
  }

  bool canCollect()
  {
    return !vehicle_dynamics_.is_moving_ && !have_gyro_bias_;
  }

  void opportunisticGyroBiasCalculator(AbstractBaseMsg* base_msg)
  {
    ImuMsg* imu = dynamic_cast<ImuMsg*>(base_msg);

    if (canCollect()) {
      if (gyro_bias_calc_.in_progress_) {
        collectGyroData(imu);

        if (gyro_bias_calc_.num_samples_ > GYRO_NUM_BIAS_SAMPLES_NEEDED) {
          gyro_bias_calc_.in_progress_ = false;

          gyro_bias_x_ = gyro_bias_calc_.sum_x_ / gyro_bias_calc_.num_samples_;
          gyro_bias_y_ = gyro_bias_calc_.sum_y_ / gyro_bias_calc_.num_samples_;
          gyro_bias_z_ = gyro_bias_calc_.sum_z_ / gyro_bias_calc_.num_samples_;
          have_gyro_bias_ = true;
          std::cerr << "Completed IMU bias: " << gyro_bias_z_ << std::endl;
        }
      } else {
        gyro_bias_calc_.in_progress_ = true;
        gyro_bias_calc_.sum_x_ = 0;
        gyro_bias_calc_.sum_y_ = 0;
        gyro_bias_calc_.sum_z_ = 0;
        gyro_bias_calc_.num_samples_ = 0;
      }
    } else {
      gyro_bias_calc_.in_progress_ = false;
    }
  }

  void updateTheta(std::shared_ptr<sensor_msgs::msg::Imu> imu_msg)
  {
    uint64_t curr_imu_time_ns = (static_cast<uint64_t>(imu_msg->header.stamp.sec) * 1000 * 1000 * 1000) + imu_msg->header.stamp.nanosec;
    if (last_imu_time_ns_ == 0) {
      last_imu_time_ns_ = curr_imu_time_ns;
      last_imu_ang_vel_z_ = imu_msg->angular_velocity.z;
      return;
    }

    if (curr_imu_time_ns < last_imu_time_ns_) {
      std::cerr << "Out of order: curr " << curr_imu_time_ns << ", last: " << last_imu_time_ns_ << std::endl;
      assert(false);
    }

    double time_diff_ns = curr_imu_time_ns - last_imu_time_ns_;

    // TODO(clalancette): Instead of a simple average we should probably bias
    // this with an acceleration.  That is, if the last reading was 0.5 and this
    // one is 0.1, we are clearly decelerating so the intervening data should
    // take that into account.

    double ang_vel_z_avg = (last_imu_ang_vel_z_ + imu_msg->angular_velocity.z) / 2;

    double rads = ang_vel_z_avg * RCL_NS_TO_S(time_diff_ns);

    theta_ += rads;
    if (imu_msg->angular_velocity.z > 0) {
      if (theta_ > JAGUAR_PI) {
        theta_ = -JAGUAR_PI + (theta_ - JAGUAR_PI);
      }
    } else {
      if (theta_ < -JAGUAR_PI) {
        theta_ = JAGUAR_PI + (JAGUAR_PI + theta_);
      }
    }

    last_imu_time_ns_ = curr_imu_time_ns;
    last_imu_ang_vel_z_ = imu_msg->angular_velocity.z;
  }

  void publishIMUMsg(AbstractBaseMsg* base_msg)
  {
    ImuMsg* imu = dynamic_cast<ImuMsg*>(base_msg);
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();

    std::pair<std::chrono::seconds, std::chrono::nanoseconds> ts = base_msg->getTime();
    imu_msg->header.stamp.sec = ts.first.count();
    imu_msg->header.stamp.nanosec = ts.second.count();
    imu_msg->orientation.x = imu->comp_x_; // currently showing RAW magnetic sensor data
    imu_msg->orientation.y = imu->comp_y_; // currently showing RAW magnetic sensor data
    imu_msg->orientation.z = imu->comp_z_; // currently showing RAW magnetic sensor data
    imu_msg->orientation.w = 0.0;

    // The ITG3205 datasheet says 14.375 deg/sec/LSB for 16g, which is
    // what we are configured for.
    imu_msg->angular_velocity.x = deg_per_sec_to_rad_per_sec((imu->gyro_x_ - gyro_bias_x_) / 14.375);
    imu_msg->angular_velocity.y = deg_per_sec_to_rad_per_sec((imu->gyro_y_ - gyro_bias_y_) / 14.375);
    imu_msg->angular_velocity.z = deg_per_sec_to_rad_per_sec((imu->gyro_z_ - gyro_bias_z_) / 14.375);

    // The ADXL345 datasheet says between 28.6 and 34.5, but empirically we
    // found we need to use 25.0 to get close to gravity.  Shrug.
    imu_msg->linear_acceleration.x = imu->accel_x_ / 25.0;
    imu_msg->linear_acceleration.y = imu->accel_y_ / 25.0;
    imu_msg->linear_acceleration.z = imu->accel_z_ / 25.0;

    updateTheta(imu_msg);

    // don't know covariances, they're defaulting to 0
    imu_pub_->publish(imu_msg);
  }

  void publishGPSMsg(AbstractBaseMsg* base_msg)
  {
    GPSMsg* gps = dynamic_cast<GPSMsg*>(base_msg);
    auto navsat_fix_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();

    std::pair<std::chrono::seconds, std::chrono::nanoseconds> ts = base_msg->getTime();
    navsat_fix_msg->header.stamp.sec = ts.first.count();
    navsat_fix_msg->header.stamp.nanosec = ts.second.count();
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

  void updatePoseXY(MotorEncPosDiffMsg *motor_enc_pos_diff)
  {
    // Ported from: https://github.com/yujinrobot/kobuki_core/blob/devel/kobuki_driver/src/driver/diff_drive.cpp#L52

    // The Jaguar4x4 has the wheel encoders inverted
    double right_diff_ticks = -motor_enc_pos_diff->pos_diff_2_;

    double left_diff_ticks = motor_enc_pos_diff->pos_diff_1_;

    ecl::LegacyPose2D<double> pose_update = diff_drive_k_.forward(RADS_PER_TICK * left_diff_ticks,
                                                                  RADS_PER_TICK * right_diff_ticks);

    std::pair<std::chrono::seconds, std::chrono::nanoseconds> ts = motor_enc_pos_diff->getTime();
    uint64_t curr_timestamp_ns = RCUTILS_S_TO_NS(ts.first.count()) + ts.second.count();
    double last_diff_time_sec{0.0};
    if (curr_timestamp_ns != last_timestamp_ns_) {
      last_diff_time_sec = (curr_timestamp_ns - last_timestamp_ns_) / 1000000000.0;
      last_timestamp_ns_ = curr_timestamp_ns;
    } else {
      std::cerr << "BUGGGGG" << std::endl;
      assert(false);
    }

    pose_update_rates_ << pose_update.x() / last_diff_time_sec,
      pose_update.y() / last_diff_time_sec,
      pose_update.heading() / last_diff_time_sec;

    // Ported from: https://github.com/ros2/turtlebot2_demo/blob/master/turtlebot2_drivers/src/kobuki_node.cpp#L156
    pose_ *= pose_update;
  }

  void updateMotorMsg(AbstractBaseMsg* base_msg)
  {
    std::lock_guard<std::mutex> sf_lock_guard(motors_sensor_frame_mutex_);
    MotorMsg* motor = dynamic_cast<MotorMsg*>(base_msg);
    jaguar4x4_comms_msgs::msg::MotorBoard* motor_ref;
    int64_t old_encoder_diff1;
    int64_t old_encoder_diff2;

    if (motor->motor_num_ == FRONT_MOTOR_NUM) {
      motor_ref = &motors_msg_->motor0;
      // Save off the current encoder diff, since it will be replaced during
      // abstractMotorToROS() below, and we want to accumulate it.
      old_encoder_diff1 = motors_msg_->motor0.encoder_diff_1;
      old_encoder_diff2 = motors_msg_->motor0.encoder_diff_2;
    } else if (motor->motor_num_ == REAR_MOTOR_NUM) {
      motor_ref = &motors_msg_->motor1;
      old_encoder_diff1 = motors_msg_->motor1.encoder_diff_1;
      old_encoder_diff2 = motors_msg_->motor1.encoder_diff_2;
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

      if (motor->motor_num_ == FRONT_MOTOR_NUM) {
        updatePoseXY(motor_enc_pos_diff);
      }

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

      if (travel_one_meter_running_ && motor->motor_num_ == FRONT_MOTOR_NUM) {
        int ticks_to_add = (motor_enc_pos_diff->pos_diff_1_ + -motor_enc_pos_diff->pos_diff_2_) / 2;
        travel_one_meter_total_ticks_ += ticks_to_add;
      }
    }

    // When all of the motors are at 0, we want to:
    // 1.  Start a timer when this first happens,
    // 2.  If the timer has gone for 5 seconds, now assume that all dynamics on
    //     the robot due to driving have stopped.  We can now take a gyro bias
    //     at any time.
    if (motors_msg_->motor0.encoder_diff_1 == 0 && motors_msg_->motor0.encoder_diff_2 == 0 &&
        motors_msg_->motor1.encoder_diff_1 == 0 && motors_msg_->motor1.encoder_diff_2 == 0) {
      if (!vehicle_dynamics_.checking_zero_) {
        vehicle_dynamics_.checking_zero_ = true;
        vehicle_dynamics_.zero_start_time_ = std::chrono::system_clock::now();
      } else {
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

        auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - vehicle_dynamics_.zero_start_time_);
        if (diff_ms.count() > VEHICLE_DYNAMICS_SETTLE_TIME_MS) {
          vehicle_dynamics_.checking_zero_ = false;
          vehicle_dynamics_.is_moving_ = false;
        }
      }
    } else {
      vehicle_dynamics_.checking_zero_ = false;
      vehicle_dynamics_.is_moving_ = true;
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

      if (base_msg) {
        switch(base_msg->getType()) {
        case AbstractBaseMsg::MessageType::imu:
          opportunisticGyroBiasCalculator(base_msg.get());
          if (have_gyro_bias_) {
            publishIMUMsg(base_msg.get());
          }
          break;
        case AbstractBaseMsg::MessageType::gps:
          publishGPSMsg(base_msg.get());
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

  int calcPWMFromTwistVels(double linear_x, double angular_z, Wheel wheel)
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

    int pwm_left = calcPWMFromTwistVels(msg->linear.x, msg->angular.z, Wheel::LEFT_WHEEL);
    int pwm_right = calcPWMFromTwistVels(msg->linear.x, msg->angular.z, Wheel::RIGHT_WHEEL);

    std::stringstream ss;

    ss << "Linear: " << msg->linear.x << ", angular: " << msg->angular.z << ", pwm_left: " << pwm_left << ", pwm_right: " << pwm_right;
    RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());

    if (prior_pwm_left_ != pwm_left || prior_pwm_right_ != pwm_right) {
      base_cmd_->move(pwm_left, -pwm_right);
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
      e_stopped_ = true;
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
      e_stopped_ = false;
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
      if (diff_ms.count() > WATCHDOG_INTERVAL_MS) {
        if (!e_stopped_ && have_gyro_bias_) {
          if (num_data_recvd_ < MIN_PINGS_EXPECTED) {
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

      std::this_thread::sleep_for(std::chrono::milliseconds(PING_TIMER_INTERVAL_MS));

      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void publishOdomMsg(rcutils_time_point_value_t now)
  {
    auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";

    // Stuff and publish /odom
    odom_msg->header.stamp.sec = RCL_NS_TO_S(now);
    odom_msg->header.stamp.nanosec = now - RCL_S_TO_NS(odom_msg->header.stamp.sec);
    odom_msg->pose.pose.position.x = pose_.x();
    odom_msg->pose.pose.position.y = pose_.y();
    odom_msg->pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose_.heading());
    odom_msg->pose.pose.orientation.x = q.x();
    odom_msg->pose.pose.orientation.y = q.y();
    odom_msg->pose.pose.orientation.z = q.z();
    odom_msg->pose.pose.orientation.w = q.w();

    for (unsigned int i = 0; i < odom_msg->pose.covariance.size(); ++i) {
      odom_msg->pose.covariance[i] = 0.0;
    }
    // Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
    // Odometry yaw covariance must be much bigger than the covariance provided
    // by the imu, as the later takes much better measures
    odom_msg->pose.covariance[0] = 0.1;
    odom_msg->pose.covariance[7] = 0.1;
    // odom_msg->pose.covariance[35] = use_imu_heading ? 0.05 : 0.2;
    odom_msg->pose.covariance[35] = 0.2;

    odom_msg->pose.covariance[14] = DBL_MAX;  // set a non-zero covariance on unused
    odom_msg->pose.covariance[21] = DBL_MAX;  // dimensions (z, pitch and roll); this
    odom_msg->pose.covariance[28] = DBL_MAX;  // is a requirement of robot_pose_ekf

    odom_msg->twist.twist.linear.x = pose_update_rates_[0];
    odom_msg->twist.twist.linear.y = pose_update_rates_[1];
    odom_msg->twist.twist.linear.z = 0.0;
    odom_msg->twist.twist.angular.x = 0.0;
    odom_msg->twist.twist.angular.y = 0.0;
    odom_msg->twist.twist.angular.z = pose_update_rates_[2];

    odom_pub_->publish(odom_msg);

    auto odom_tf_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
    odom_tf_msg->header.frame_id = "odom";
    odom_tf_msg->child_frame_id = "base_link";

    // Stuff and publish /tf
    odom_tf_msg->header.stamp = odom_msg->header.stamp;
    odom_tf_msg->transform.translation.x = pose_.x();
    odom_tf_msg->transform.translation.y = pose_.y();
    odom_tf_msg->transform.translation.z = 0.0;
    odom_tf_msg->transform.rotation.x = q.x();
    odom_tf_msg->transform.rotation.y = q.y();
    odom_tf_msg->transform.rotation.z = q.z();
    odom_tf_msg->transform.rotation.w = q.w();

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(*odom_tf_msg);
    tf_pub_->publish(tf_msg);
  }

  void motorsPubThread(std::shared_future<void> local_future)
  {
    std::future_status status;

    do {
      // to set timestamp
      rcutils_time_point_value_t now;
      if (rcutils_system_time_now(&now) == RCUTILS_RET_OK) {
        if (new_motor_data_) {
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
            copyMotorBoardMessage(&(motors_pub_msg->motor0), &(motors_msg_->motor0));
            copyMotorBoardMessage(&(motors_pub_msg->motor1), &(motors_msg_->motor1));

            motors_msg_->motor0.encoder_diff_1 = 0;
            motors_msg_->motor0.encoder_diff_2 = 0;

            motors_msg_->motor1.encoder_diff_1 = 0;
            motors_msg_->motor1.encoder_diff_2 = 0;

            new_motor_data_ = false;
          }

          motors_pub_->publish(motors_pub_msg);
        }

        publishOdomMsg(now);
      } else {
        std::cerr << "unable to access time\n";
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(MOTORS_PUB_TIMER_INTERVAL_MS));

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
    response->rpms = 0.0;
    response->speed_m_per_s = 0.0;

    if (!accepting_commands_) {
      response->success = false;
      response->message = "Not accepting commands (eStopped?)";
      return;
    }

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
      response->success = false;
      response->message = "Invalid motor requested";
      return;
    }

    // Reset our counters
    enc_count_service_.counts_ = 0;

    // Start the base moving
    auto start_time = std::chrono::high_resolution_clock::now();
    base_cmd_->move(request->test_motor_power, -request->test_motor_power);
    enc_count_service_.running_ = true;

    response->success = true;
    response->message = "Success";

    std::chrono::duration<double, std::milli> diff_ms;
    diff_ms = std::chrono::high_resolution_clock::now() - start_time;
    while (diff_ms.count() < request->test_duration_ms) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      // After waking up, we always check to make sure the robot is still
      // accepting commands (this could have changed either via a network
      // disconnect or from an eStop command coming in from the joystick).
      // If it isn't accepting commands anymore, fail the service.
      if (!accepting_commands_) {
        response->success = false;
        response->message = "Stopped accepting commands (eStopped?)";
        break;
      }
      diff_ms = std::chrono::high_resolution_clock::now() - start_time;
    }

    enc_count_service_.running_ = false;
    // Stop the base movement
    base_cmd_->move(0, 0);

    if (response->success) {
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
      // circumference (we know the Jaguar4x4 wheel radius is 0.13 meters).
      //
      // Given that, we can now determine the speed in m/s by the following
      // formula:
      //
      // speed_m_per_s = rpms * circum_m / 60
      //
      // (rpms * circum_m determines the speed in meters/minute, then we divide
      // by 60 to get the speed in meters/second).
      response->speed_m_per_s = response->rpms * JAGUAR_WHEEL_CIRCUM_M / 60.0;
    }
  }

  void travel_one_meter(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (!accepting_commands_) {
      response->success = false;
      response->message = "Not accepting commands (eStopped?)";
      return;
    }

    int pwm_left = calcPWMFromTwistVels(0.2, 0.0, Wheel::LEFT_WHEEL);
    int pwm_right = calcPWMFromTwistVels(0.2, 0.0, Wheel::LEFT_WHEEL);

    travel_one_meter_running_ = true;
    travel_one_meter_total_ticks_ = 0;

    base_cmd_->move(pwm_left, -pwm_right);

    response->success = true;
    response->message = "go go go";

    double total_moved = 0.0;
    while (total_moved < 1.0) {
      double percent_wheel_traveled = travel_one_meter_total_ticks_ / JAGUAR_ENCODER_TICKS;
      total_moved = JAGUAR_WHEEL_CIRCUM_M * percent_wheel_traveled;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      // After waking up, we always check to make sure the robot is still
      // accepting commands (this could have changed either via a network
      // disconnect or from an eStop command coming in from the joystick).
      // If it isn't accepting commands anymore, fail the service.
      if (!accepting_commands_) {
        response->success = false;
        response->message = "Stopped accepting commands (eStopped?)";
        break;
      }
    }

    base_cmd_->move(0, 0);

    travel_one_meter_running_ = false;
  }

  struct Point final
  {
    double speed_m_per_s_;
    double pwm_;
  };

  // Tunable parameters
  const uint32_t FRONT_MOTOR_NUM = 0;
  const uint32_t REAR_MOTOR_NUM = 1;
  const uint32_t MOTORS_PUB_TIMER_INTERVAL_MS = 100;
  const uint32_t PING_TIMER_INTERVAL_MS = 40;
  const uint32_t WATCHDOG_INTERVAL_MS = 200;
  static constexpr double PING_RECV_PERCENTAGE = 0.8;
  const double JAGUAR_WHEEL_RADIUS_M = 0.1325;
  static constexpr double JAGUAR_WHEEL_BASE_M = 0.35;
  static constexpr double JAGUAR_BASE_ENCODER_COUNTS_PER_REVOLUTION = 520.0;
  static constexpr int GYRO_NUM_BIAS_SAMPLES_NEEDED = 100;
  static constexpr uint32_t VEHICLE_DYNAMICS_SETTLE_TIME_MS = 3000;
  const double JAGUAR_ENCODER_TICKS = 520.0;
  const double JAGUAR_AXLE_LENGTH_M = 0.425;
  const Point PWM_TO_SPEED_START_POINT_{0.8840964279, 500};
  const Point PWM_TO_SPEED_END_POINT_{0.1063000495, 95};
  const double JAGUAR_PI = 3.14159265;

  // Constants calculated based on the tunable parameters above
  const double JAGUAR_TWO_PI = 2.0 * JAGUAR_PI;
  const uint32_t PINGS_PER_WATCHDOG_INTERVAL = WATCHDOG_INTERVAL_MS / PING_TIMER_INTERVAL_MS;
  const uint32_t MIN_PINGS_EXPECTED = PINGS_PER_WATCHDOG_INTERVAL * PING_RECV_PERCENTAGE;
  const double RADS_PER_TICK = JAGUAR_TWO_PI / JAGUAR_ENCODER_TICKS;
  const double JAGUAR_WHEEL_CIRCUM_M = JAGUAR_TWO_PI * JAGUAR_WHEEL_RADIUS_M;

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
  std::atomic<bool>                                                       e_stopped_{true};
  struct EncoderCountService final
  {
    int counts_;
    std::atomic_bool running_;
    int motor_board_num_;
    int motor_num_;
  };
  EncoderCountService                                                     enc_count_service_;

  struct VehicleDynamics final
  {
    bool is_moving_{true};
    bool checking_zero_{false};
    std::chrono::time_point<std::chrono::system_clock> zero_start_time_;
  };
  VehicleDynamics                                                           vehicle_dynamics_;

  struct GyroBiasCalc final
  {
    int sum_x_{0};
    int sum_y_{0};
    int sum_z_{0};
    int num_samples_{0};
    bool in_progress_{false};
  };
  GyroBiasCalc                                                            gyro_bias_calc_;

  // These are the outputs from the GyroBiasCalc above.
  int                                                                     gyro_bias_x_{0};
  int                                                                     gyro_bias_y_{0};
  int                                                                     gyro_bias_z_{0};
  bool                                                                    have_gyro_bias_{false};
  ecl::DifferentialDrive::Kinematics                                      diff_drive_k_{JAGUAR_AXLE_LENGTH_M, JAGUAR_WHEEL_RADIUS_M};
  uint64_t                                                                last_timestamp_ns_{0};
  ecl::linear_algebra::Vector3d                                           pose_update_rates_;
  ecl::LegacyPose2D<double>                                               pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                   odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr                  tf_pub_;
  double                                                                  theta_{0.0};
  uint64_t                                                                last_imu_time_ns_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                      travel_one_meter_srv_;
  std::atomic<bool>                                                       travel_one_meter_running_{false};
  std::atomic<int>                                                        travel_one_meter_total_ticks_{0};
  double                                                                  last_imu_ang_vel_z_{0.0};
  rclcpp::callback_group::CallbackGroup::SharedPtr                        joy_cb_grp_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto base = std::make_shared<Jaguar4x4Base>("192.168.0.60", 10001);
  executor.add_node(base);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
