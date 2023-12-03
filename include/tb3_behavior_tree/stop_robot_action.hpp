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


bool stop_flag = 0;

//-------------------------------------------------------------------------------------
//------------------------------ Class Stop ---------------------------------------
//-------------------------------------------------------------------------------------
class Stop : public BT::SyncActionNode, public rclcpp::Node {
  private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    int neuron_num = 6;

    void stop_callback(const std_msgs::msg::Int32::SharedPtr stop_msg) {
      int stop_msg_data = stop_msg->data;
      if(stop_msg_data == 3){
        stop_flag = 1;
      }
      else{ stop_flag = 0; }

    }
  public:
    Stop(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), Node("stop_node") {
      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/laser_condition", sensor_qos,
        [&](const std_msgs::msg::Int32::SharedPtr msg) {
          stop_callback(msg);
        }
      );
    }

    BT::NodeStatus tick() override {
      publisher_ =
          this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      if (stop_flag){
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;
        message.linear.z= 0.0;
        publisher_->publish(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      }

      return (stop_flag) ? NodeStatus::SUCCESS
                        : NodeStatus::FAILURE;
    }
    static PortsList providedPorts() { return {}; }
};
