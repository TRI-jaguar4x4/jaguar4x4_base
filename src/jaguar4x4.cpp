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
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str())
    publisher_->publish(message);

    struct MotorSensorData motorSensorData_;
    struct MotorBoardData motorBoardData_;
    struct IMUSensorData imuSensorData_;
    struct GPSSensorData gpsSensorData_;
    sensorDriver.readMotorSensorData(&motorSensorData_);
    sensorDriver.readMotorBoardData(&motorBoardData_);
    sensorDriver.readIMUSensorData(&imuSensorData_);
    sensorDriver.readGPSSensorData(&gpsSensorData_);
    RCLCPP_INFO(this->get_logger(), "YAW: %f\n", imuSensorData_.yaw);
    rcutils_time_point_value_t now;
    rcutils_system_time_now(&now);
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp.sec=RCL_NS_TO_S(now);
    imu_msg->header.stamp.nanosec=now - RCL_S_TO_NS(imu_msg->header.stamp.sec);
    imu_msg->orientation.x = 0.0;
    imu_msg->orientation.y = 0.0;
    imu_msg->orientation.z = 0.0;
    imu_msg->orientation.w = 0.0;
    imu_msg->angular_velocity.x = 0.0;
    imu_msg->angular_velocity.y = 0.0;
    imu_msg->angular_velocity.z = 0.0;
    imu_msg->linear_acceleration.x = 0.0;
    imu_msg->linear_acceleration.y = 0.0;
    imu_msg->linear_acceleration.z = 9.8;
    // don't know covariances, they're defaulting to 0
    imuPub->publish(imu_msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  DrRobotMotionSensorDriver sensorDriver;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jaguar4x4>());
  rclcpp::shutdown();
  return 0;
}
