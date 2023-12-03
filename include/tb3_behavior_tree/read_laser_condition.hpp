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
#include <sensor_msgs/msg/laser_scan.hpp>

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

//------------------------------------------------------------------------------------
//----------------------------------- Global  ----------------------------------------
//------------------------------------------------------------------------------------
int laser_status;  // Stop: 3, Right: 2, Left: 1

//-------------------------------------------------------------------------------------
//------------------------------ Class ReadingLaser -----------------------------------
//-------------------------------------------------------------------------------------
class ReadingLaser : public BT::ConditionNode, public rclcpp::Node {
  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
      int scan_len = scan_msg->ranges.size();

      // Front 0° to 180°
      int start_index = scan_len / 4;       // Start Index
      int end_index = (3 * scan_len) / 4;   // Goal Index
      std::vector<float> extracted_data;
      for (int i = 0; i < scan_len; i++) {
        if (i < start_index || i >= end_index) {
          extracted_data.push_back(scan_msg->ranges[i]);
        }
      }
      scan_len = extracted_data.size();
      int sectionSize = scan_len / neuron_num;
      int startIndex = 0;

      float flag_thre = 3.0;
      for (int i = 0; i < neuron_num; i++)
      {
        int endIndex = startIndex + sectionSize;
        if (i == neuron_num-1){
          endIndex = scan_len;
        }
        // neuron range vector value
        std::vector<float> section(
          extracted_data.begin() + startIndex, extracted_data.begin() + endIndex
        );

        // calculate min range [mm]
        in_min_data[i] = section[0];
        for (const auto& range : section){
          if (!std::isnan(range) && std::isfinite(range)){
            if(range < in_min_data[i]){
              in_min_data[i] = range;
            }
          }
        }

        RCLCPP_INFO(rclcpp::get_logger("scan_debug"), "Laser data %d: %f", i, in_min_data[i]);
        startIndex = endIndex;
      }

      min_value = in_min_data[0];
      for (int i = 0; i < neuron_num; i++) {
        if (in_min_data[i] < min_value) {
          min_value = in_min_data[i];
          min_index = i;
        }
      }
      RCLCPP_INFO(rclcpp::get_logger("scan_debug"), "Neuron Min Index: %d, Value %f", min_index, min_value);

      // Stop
      if ((min_value < 0.8)){
        laser_status = 3; // Stop
        RCLCPP_INFO(rclcpp::get_logger("scan_debug"), "Stop!, Status: {%d}", laser_status);
      }
      // Left
      else if ((min_value < flag_thre) && (min_index < neuron_num/2)){
          laser_status = 1;
          RCLCPP_INFO(rclcpp::get_logger("scan_debug"), "Left attention!, Status: {%d}", laser_status);
      }
      // Right
      else if ((min_value < flag_thre) && (min_index >= neuron_num/2)){
        laser_status = 2;
        RCLCPP_INFO(rclcpp::get_logger("scan_debug"), "Right attention!, Status: {%d}", laser_status);
      }
      else{ laser_status = 0; }
    }

  public:
    int neuron_num = 6;
    std::vector<float> in_min_data;
    int min_index = 0;
    float min_value;
    ReadingLaser(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), Node("laser_node") {
      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      in_min_data.resize(neuron_num);

      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", sensor_qos,
        [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          scan_callback(msg);
        }
      );
    }

    BT::NodeStatus tick() override {
      publisher_ =
          this->create_publisher<std_msgs::msg::Int32>("/laser_condition", 10);
      auto status = std_msgs::msg::Int32();
      status.data = laser_status;
      publisher_->publish(status);
      setOutput("laser_status", int(laser_status));

      RCLCPP_INFO(this->get_logger(), "This is a tick()-------");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return (laser_status != 0 ) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    }
    static PortsList providedPorts() {
      const char *description = "Simply print the target on console...";
      return {
        OutputPort<int>("laser_status", description)
      };
    }
};
