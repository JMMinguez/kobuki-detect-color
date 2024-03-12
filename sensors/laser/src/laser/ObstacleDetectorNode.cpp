// Copyright 2024 Intelligent Robotics Lab
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

#include <memory>

#include "laser/ObstacleDetectorNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3.hpp"

namespace laser
{

using std::placeholders::_1;

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node")
{
  declare_parameter("min_distance", min_distance_);
  get_parameter("min_distance", min_distance_);

  RCLCPP_INFO(get_logger(), "ObstacleDetectorNode set to %f m", min_distance_);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS().reliable(),
    std::bind(&ObstacleDetectorNode::laser_callback, this, _1));
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>(
    "obstacle", 100);

  r_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3>("repulsive_vector", 10);

}

void
ObstacleDetectorNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  int min_idx = std::min_element(scan->ranges.begin(), scan->ranges.end()) - scan->ranges.begin();
  float distance_min = scan->ranges[min_idx];

  auto obstacle_msg = std_msgs::msg::Bool();

  if (distance_min < 1.0) {
    float angle = scan->angle_min + scan->angle_increment * min_idx;
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}

    RCLCPP_INFO(get_logger(), "Obstacle in (%f, %f)", distance_min, angle);

    float repulsion = 1.0 / distance_min;
    obstacle_msg.data = true;
    repulsive_vector_msg.x = repulsion * cos(angle);
    repulsive_vector_msg.y = repulsion * sin(angle);
    repulsive_vector_msg.z = 0.0;

  } else {
    obstacle_msg.data = false;

    repulsive_vector_msg.x = 0.0;
    repulsive_vector_msg.y = 0.0;
    repulsive_vector_msg.z = 0.0;
  }

  std::cerr << "X: \t" << repulsive_vector_msg.x << " Y: \t" << repulsive_vector_msg.y <<
    std::endl;

  r_vector_pub_->publish(repulsive_vector_msg);
  obstacle_pub_->publish(obstacle_msg);
}

}  // namespace laser
