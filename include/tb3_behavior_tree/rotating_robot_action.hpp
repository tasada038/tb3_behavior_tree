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
//---------------------ROTATING_ROBOT--------------------------------------------------
//-------------------------------------------------------------------------------------
class Rotating : public BT::SyncActionNode, public rclcpp::Node {
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // void rotating_callback(const std_msgs::msg::Int32::SharedPtr rot_msg) {
    //   int rot_data = rot_msg->data;
    // }

  public:
    Rotating(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), Node("rotating_node") {
    }

    // NodeStatus onStart() { return NodeStatus::RUNNING; }
    // NodeStatus onRunning() { return NodeStatus::FAILURE; }
    NodeStatus tick() override {
      auto res_time = getInput<int>("sleep_mtime");
      auto res_x = getInput<float>("linear_x");
      auto res_status = getInput<int>("laser_status");
      if (!res_time) {
        throw RuntimeError("error reading port [sleep_mtime]:", res_time.error());
      }
      int sleep_mtime = res_time.value();
      float linear_x = res_x.value();
      int rot_data = res_status.value();

      publisher_ =
          this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      auto message = geometry_msgs::msg::Twist();

      RCLCPP_INFO(this->get_logger(), "rot_data: %d", rot_data);
      if (rot_data == 1) {
          message.angular.z = -0.3;
      }
      if (rot_data == 2) {
          message.angular.z = 0.3;
      }
      if (rot_data == 3) {
        message.angular.z = 0.0;
      }
      message.linear.x = linear_x;
      publisher_->publish(message);

      RCLCPP_INFO(this->get_logger(), "This is a tick()-------");
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_mtime));
      return (rot_data != 0 ) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

    static PortsList providedPorts() {

      const char *description = "Simply print the target on console...";
      return {
        InputPort<int>("sleep_mtime", description),
        InputPort<float>("linear_x", description),
        InputPort<int>("laser_status", description),
      };
    }

};