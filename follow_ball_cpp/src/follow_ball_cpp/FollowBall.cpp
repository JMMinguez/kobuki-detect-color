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
  r_vector_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "repulsive_vector", 10,
    std::bind(&FollowBall::r_vector_callback, this, _1));
  a_vector_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "attractive_vector", 10,
    std::bind(&FollowBall::a_vector_callback, this, _1));

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_ = create_wall_timer(
    50ms, std::bind(&FollowBall::follow_objective, this));

  object_sub_ = create_subscription<std_msgs::msg::Bool>(
    "object", 10,
    std::bind(&FollowBall::object_callback, this, _1));
}

void
FollowBall::r_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  last_repulsive_vector_ = *msg;
}

void
FollowBall::a_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  last_attractive_vector_ = *msg;
}

void
FollowBall::object_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_object = *msg;
}

void
FollowBall::follow_objective()
{
  switch (state_) {
    case FOLLOW:
      objective_vector_.x = last_attractive_vector_.x + last_repulsive_vector_.x;
      objective_vector_.y = last_attractive_vector_.y + last_repulsive_vector_.y;

      current_vel_.linear.x = objective_vector_.x;
      current_vel_.angular.z = objective_vector_.y;

      vel_pub_->publish(current_vel_);

      break;
    
    case RIGHT_TURN:
      break;
    
    case LEFT_TURN:
      break;
  }
    

}
}  //  namespace follow_ball_cpp