//-----------------------------------------------------------------------------------
// MIT License

// Copyright (c) 2023 Takumi Asada

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//-----------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
//----------------------------------- Include ----------------------------------------
//------------------------------------------------------------------------------------
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/tree_node.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <iostream>

using namespace std::chrono_literals;
using std::chrono::milliseconds;
using std::placeholders::_1;

using namespace BT;

//-------------------------------------------------------------------------------------
//--------------------------Class MOVE_ROBOT-------------------------------------------
//-------------------------------------------------------------------------------------
class MoveRobot : public BT::SyncActionNode, public rclcpp::Node {
  private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr _msg) {

      tf2::Quaternion quat_tf;
      geometry_msgs::msg::Quaternion quat_msg = _msg->pose.pose.orientation;
      tf2::fromMsg(quat_msg, quat_tf);
      double roll{}, pitch{}, yaw{};
      tf2::Matrix3x3 m(quat_tf);
      m.getRPY(roll, pitch, yaw);

      // Left ＋,  Right −
      float yaw_check = yaw * 180 / M_PI;

      float robotAngle = yaw_check;
      RCLCPP_INFO(this->get_logger(), "yaw: '%f'", robotAngle);
      RCLCPP_INFO(this->get_logger(), "position: '%f' '%f'",
                  _msg->pose.pose.position.x, _msg->pose.pose.position.y);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  public:
    MoveRobot(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), Node("move_node") {

      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom", sensor_qos, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            odometry_callback(msg);
          });
    }

    NodeStatus tick() override {
      auto res_time = getInput<int>("sleep_mtime");
      if (!res_time) {
        throw RuntimeError("error reading port [sleep_mtime]:", res_time.error());
      }
      int sleep_mtime = res_time.value();

      publisher_ =
          this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      auto message = geometry_msgs::msg::Twist();

      message.linear.x = 0.2;
      setOutput("linear_x", float(message.linear.x));

      publisher_->publish(message);

      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_mtime));

      return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts() {
      const char *description = "Simply print the target on console...";
      return {
        InputPort<int>("sleep_mtime", description),
        OutputPort<float>("linear_x", description)
      };
    }
};