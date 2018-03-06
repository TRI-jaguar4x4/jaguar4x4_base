// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "jaguar4x4_msgs/msg/motor_board.hpp" // pattern is all lower case name w/ underscores
#include "DrRobotMotionSensorDriver.hpp"

using namespace std::chrono_literals;
using namespace DrRobot_MotionSensorDriver;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Jaguar4x4 : public rclcpp::Node
{
public:
  Jaguar4x4()
  : Node("jaguar4x4"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Jaguar4x4::timer_callback, this));

    struct DrRobotMotionConfig motionConfig;
    std::string str("192.168.0.60");
    str.copy(motionConfig.robotIP, str.length());
    motionConfig.robotIP[str.length()]='\0';
    motionConfig.portNum=10001;
    motionConfig.commMethod = Network;
    motionConfig.robotType = Jaguar;

    sensorDriver.setDrRobotMotionDriverConfig(&motionConfig);

    int ret = sensorDriver.openNetwork(motionConfig.robotIP, motionConfig.portNum);
    if (ret) {
      RCLCPP_WARN(get_logger(), "openNetwork failed %s", motionConfig.robotIP);
    }

    imuPub = this->create_publisher<sensor_msgs::msg::Imu>("imu");
    navsatPub = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsat");
    motorBoardBase1Pub = this->create_publisher<jaguar4x4_msgs::msg::MotorBoard>("motorboardBase1");
    motorBoardBase2Pub = this->create_publisher<jaguar4x4_msgs::msg::MotorBoard>("motorboardBase2");

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
 
    cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
								     std::bind(&Jaguar4x4::cmdVelCallback, this, std::placeholders::_1), cmd_vel_qos_profile);

  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto message = std_msgs::msg::String();
    message.data = "Time to operate the robot! " + std::to_string(count_++);
    RCLCPP_DEBUG(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    float linearX = msg->linear.x*350;
    float angularZ = msg->angular.z*700;
    
    // for now, this will hold up publication... move to its own thread
    sensorDriver.sendCommand("MMW !MG", 7);
    if (linearX != 0.0) {
      std::stringstream ss; 
      ss << "MMW !M " << linearX << " " << -linearX;
      sensorDriver.sendCommand(ss.str().c_str(), ss.str().length());
    }

    if (angularZ != 0.0) {
      std::stringstream ss; 
      ss << "MMW !M " << angularZ << " " << angularZ;
      sensorDriver.sendCommand(ss.str().c_str(), ss.str().length());
    }

    if (linearX == 0.0 && angularZ == 0.0) { // send stop
      sensorDriver.sendCommand("MMW !EX", 7);
    }
  }
  
  void timer_callback()
  {
    struct MotorSensorData motorSensorData_;
    struct MotorBoardData motorBoardData_;
    struct IMUSensorData imuSensorData_;
    struct GPSSensorData gpsSensorData_;
    sensorDriver.readMotorSensorData(&motorSensorData_);
    sensorDriver.readMotorBoardData(&motorBoardData_);
    sensorDriver.readIMUSensorData(&imuSensorData_);
    sensorDriver.readGPSSensorData(&gpsSensorData_);
    RCLCPP_DEBUG(this->get_logger(), "YAW: %f, GPS Status %d\n",
		imuSensorData_.yaw, gpsSensorData_.gpsStatus);
    rcutils_time_point_value_t now;
    rcutils_system_time_now(&now);

    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    auto navsat_fix_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
    auto motor_board_base1_msg = std::make_shared<jaguar4x4_msgs::msg::MotorBoard>();
    auto motor_board_base2_msg = std::make_shared<jaguar4x4_msgs::msg::MotorBoard>();
    
    imu_msg->header.stamp.sec=RCL_NS_TO_S(now);
    imu_msg->header.stamp.nanosec=now - RCL_S_TO_NS(imu_msg->header.stamp.sec);
    imu_msg->orientation.x = imuSensorData_.comp_x; // currently showing RAW magnetic sensor data
    imu_msg->orientation.y = imuSensorData_.comp_y; // currently showing RAW magnetic sensor data
    imu_msg->orientation.z = imuSensorData_.comp_z; // currently showing RAW magnetic sensor data
    imu_msg->orientation.w = 0.0;
    imu_msg->angular_velocity.x = imuSensorData_.gyro_x/14.375; // all of the below are first approximation based on LSB
    imu_msg->angular_velocity.y = imuSensorData_.gyro_y/14.375;
    imu_msg->angular_velocity.z = imuSensorData_.gyro_z/14.375;
    imu_msg->linear_acceleration.x = imuSensorData_.accel_x/32.0;
    imu_msg->linear_acceleration.y = imuSensorData_.accel_y/32.0;
    imu_msg->linear_acceleration.z = imuSensorData_.accel_z/32.0;
    // don't know covariances, they're defaulting to 0

    // GPS
    navsat_fix_msg->header.stamp.sec=RCL_NS_TO_S(now);
    navsat_fix_msg->header.stamp.nanosec=now - RCL_S_TO_NS(imu_msg->header.stamp.sec);
    switch (gpsSensorData_.gpsStatus) {
      case -1: // invalid
	navsat_fix_msg->status.status=sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
	break;
      case 0:  // fixed
	navsat_fix_msg->status.status=sensor_msgs::msg::NavSatStatus::STATUS_FIX;
	break;
      case 1:  // differential (assuming satellite based differential - vs ground based)
	navsat_fix_msg->status.status=sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
	break;
      default:
        RCLCPP_WARN(get_logger(), "Invalid GPS status %d", gpsSensorData_.gpsStatus);
	break;
    }
    navsat_fix_msg->status.service=sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    navsat_fix_msg->latitude = gpsSensorData_.latitude;
    navsat_fix_msg->longitude = gpsSensorData_.longitude;
    navsat_fix_msg->altitude = 0.0;
    navsat_fix_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    // motor boards
    //motor_board_base1(2)_msg motorBoardData_
    motor_board_base1_msg->status = motorBoardData_.status[0];
    motor_board_base2_msg->status = motorBoardData_.status[1];
    motor_board_base1_msg->temp1 = motorBoardData_.temp2[0];
    motor_board_base2_msg->temp1 = motorBoardData_.temp2[1];
    motor_board_base1_msg->temp2 = motorBoardData_.temp3[0];
    motor_board_base2_msg->temp2 = motorBoardData_.temp3[1];
    motor_board_base1_msg->volt_main = motorBoardData_.volMain[0];
    motor_board_base2_msg->volt_main = motorBoardData_.volMain[1];
    motor_board_base1_msg->volt_12v = motorBoardData_.vol12V[0];
    motor_board_base2_msg->volt_12v = motorBoardData_.vol12V[1];
    motor_board_base1_msg->volt_5v = motorBoardData_.vol5V[0];
    motor_board_base2_msg->volt_5v = motorBoardData_.vol5V[1];    
      
    imuPub->publish(imu_msg);
    navsatPub->publish(navsat_fix_msg);
    motorBoardBase1Pub->publish(motor_board_base1_msg);
    motorBoardBase2Pub->publish(motor_board_base2_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  DrRobotMotionSensorDriver sensorDriver;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatPub;
  rclcpp::Publisher<jaguar4x4_msgs::msg::MotorBoard>::SharedPtr motorBoardBase1Pub;  // base motor board 1?
  rclcpp::Publisher<jaguar4x4_msgs::msg::MotorBoard>::SharedPtr motorBoardBase2Pub;  // base motor board 2?

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jaguar4x4>());
  rclcpp::shutdown();
  return 0;
}
