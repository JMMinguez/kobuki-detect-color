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
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include "follow_ball_cpp/FollowBall.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace follow_ball_cpp
{
FollowBall::FollowBall()
: Node("follow_ball")
{
  declare_parameter("max_vel", 0.75);
  declare_parameter("min_vel", 0.0);
  declare_parameter("turn_right_vel", -0.3);
  declare_parameter("turn_left_vel", 0.3);
  
  get_parameter("max_vel", max_vel_);
  get_parameter("min_vel", min_vel_);
  get_parameter("turn_right_vel", turn_right_vel_);
  get_parameter("turn_left_vel", turn_left_vel_);

  r_vector_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "repulsive_vector", 10,
    std::bind(&FollowBall::r_vector_callback, this, _1));
  a_vector_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "attractive_vector", 10,
    std::bind(&FollowBall::a_vector_callback, this, _1));

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_ = create_wall_timer(
    100ms, std::bind(&FollowBall::follow_objective, this));

  object_sub_ = create_subscription<std_msgs::msg::Bool>(
    "object", 10,
    std::bind(&FollowBall::object_callback, this, _1));
}

void
FollowBall::r_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  last_repulsive_vector_ = *msg;
  std::cerr << "r_vector: \t" << last_repulsive_vector_.x << std::endl;
  std::cerr << "r_vector: \t" << last_repulsive_vector_.y << std::endl;

}

void
FollowBall::a_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  last_attractive_vector_ = *msg;
  std::cerr << "a_vector: \t" << last_attractive_vector_.x << std::endl;
  std::cerr << "a_vector: \t" << last_attractive_vector_.y << std::endl;
  if (is_object.data) {
    if (last_attractive_vector_.x >= 0) {
        lost_right_ = true;
    } else {
        lost_right_ = false;
    }
  }
}

void
FollowBall::object_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_object = *msg;
  std::cerr << "OBJECT: \t" << is_object.data << std::endl;
}

void
FollowBall::follow_objective()
{
  switch (state_) {
    case FOLLOW:
      objective_vector_.x = std::abs(last_attractive_vector_.x + last_repulsive_vector_.x);
      objective_vector_.y = last_attractive_vector_.y + last_repulsive_vector_.y;

      current_vel_.linear.x = sqrt(objective_vector_.x * objective_vector_.x + objective_vector_.y * objective_vector_.y);
      current_vel_.angular.z = atan2(objective_vector_.y, objective_vector_.x);

      current_vel_.linear.x = std::clamp(current_vel_.linear.x, min_vel_, max_vel_);
      current_vel_.angular.z = std::clamp(current_vel_.angular.z, min_vel_, max_vel_);
      
      std::cerr << "velx: \t" << current_vel_.linear.x << std::endl;
      std::cerr << "velz: \t" << current_vel_.angular.z << std::endl;

      std::cerr << "FOLLOW: \t" << std::endl;

      vel_pub_->publish(current_vel_);
      if (!is_object.data) {
        check_turn();
      }
      break;
    
    case RIGHT_TURN:
      std::cerr << "RIGHT: \t" << std::endl;
      current_vel_.linear.x = 0;
      current_vel_.angular.z = turn_right_vel_;

      vel_pub_->publish(current_vel_);
      if (is_object.data) {
        go_state(FOLLOW);
      }
      break;
    
    case LEFT_TURN:
      std::cerr << "LEFT: \t" << std::endl;
      current_vel_.linear.x = 0;
      current_vel_.angular.z = turn_left_vel_;

      vel_pub_->publish(current_vel_);
      if (is_object.data) {
        go_state(FOLLOW);
      }
      break;
  }
  last_repulsive_vector_.x = 0.0;
  last_repulsive_vector_.y = 0.0;
}

void
FollowBall::go_state(int new_state)
{
  state_ = new_state;
}

void
FollowBall::check_turn()
{
  if (lost_right_) {
    go_state(RIGHT_TURN);
  } else {
    go_state(LEFT_TURN);
  }
}
}  //  namespace follow_ball_cpp