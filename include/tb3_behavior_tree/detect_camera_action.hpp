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
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

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
//-------------------------- Class CameraDetected -------------------------------------
//-------------------------------------------------------------------------------------
class CameraDetected : public BT::SyncActionNode, public rclcpp::Node {
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // TODO TBD
        cv::Scalar lower_target_color1(20, 100, 100); // 下限値 (Hue, Saturation, Value)
        cv::Scalar upper_target_color1(30, 255, 255);  // 上限値 (Hue, Saturation, Value)

        cv::Scalar lower_target_color2(160, 100, 100); // 別の色の下限値
        cv::Scalar upper_target_color2(180, 255, 255); // 別の色の上限値

        cv::Mat hsv_img;
        cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);

        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv_img, lower_target_color1, upper_target_color1, mask1);
        cv::inRange(hsv_img, lower_target_color2, upper_target_color2, mask2);
        mask = mask1 + mask2;

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            double max_area = 0;
            int max_area_idx = -1;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area) {
                    max_area = area;
                    max_area_idx = i;
                }
            }

            if (max_area_idx != -1) {
                cv::Rect bounding_rect = cv::boundingRect(contours[max_area_idx]);
                geometry_msgs::msg::Pose2D target_pose;
                target_pose.x = ((bounding_rect.x + bounding_rect.width / 2) - msg->width / 2) / (msg->width / 2);
                target_pose.y = ((bounding_rect.y + bounding_rect.height / 2) - msg->height / 2) / (msg->height / 2);
                target_pose.theta = 0;
                publisher_->publish(target_pose);

                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.2;
                cmd_vel.angular.z = 0.5 * atan2(target_pose.y, target_pose.x);
                cmd_vel_publisher_->publish(cmd_vel);

            }
        }
    }

  public:
    CameraDetected(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), Node("camera_node") {

        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10, std::bind(&CameraDetected::imageCallback, this, std::placeholders::_1)
        );
    }

    NodeStatus tick() override {

      publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/target_pose", 10);
      cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      // return NodeStatus::SUCCESS;
      return NodeStatus::FAILURE;
    }


    static PortsList providedPorts() { return {}; }
};