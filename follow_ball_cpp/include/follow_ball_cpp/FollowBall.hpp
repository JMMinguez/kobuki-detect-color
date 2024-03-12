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

#ifndef FOLLOW_BALL_CPP__FOLLOWBALL_HPP_
#define FOLLOW_BALL_CPP__FOLLOWBALL_HPP_

namespace follow_ball_cpp
{
class FollowBall : public rclcpp::Node
{
public:
  FollowBall();

private:
  void r_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void a_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void object_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void follow_objective();
  void go_state(int new_state);
  void check_turn();

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr r_vector_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr a_vector_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr object_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  geometry_msgs::msg::Twist current_vel_;

  geometry_msgs::msg::Vector3 last_repulsive_vector_;
  geometry_msgs::msg::Vector3 last_attractive_vector_;
  geometry_msgs::msg::Vector3 objective_vector_;
  rclcpp::TimerBase::SharedPtr timer_;

  std_msgs::msg::Bool is_object;

  static const int FOLLOW = 0;
  static const int RIGHT_TURN = 1;
  static const int LEFT_TURN = 2;

  double turn_right_vel_, turn_left_vel_, max_vel_, min_l_vel_, min_a_vel_;
  int state_;
  bool lost_right_;
};

}  //  namespace follow_ball_cpp

#endif  // FOLLOW_BALL_CPP__FOLLOWBALL_HPP_
