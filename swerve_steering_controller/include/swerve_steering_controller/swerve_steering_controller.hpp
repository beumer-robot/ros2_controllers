/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Mark Naeem
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The names of the contributors may NOT be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Mark Naeem
 */

#pragma once

#include <cmath>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
// #include <hardware_interface/joint_command_interface.h>
// #include <hardware_interface/loans.hpp>  // ROS 2 uses a different approach for hardware interfaces
#include "realtime_tools/realtime_box.h"
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
// #include <tf/tf.h>
#include <tf2_msgs/msg/tf_message.hpp>


#include <swerve_steering_controller/odometry.hpp>
#include <swerve_steering_controller/speed_limiter.hpp>
#include <swerve_steering_controller/utils.hpp>
#include <swerve_steering_controller/wheel.hpp>
#include <swerve_steering_controller/visibility_control.h>
// #include "swerve_steering_controller_parameters.hpp"

namespace swerve_steering_controller
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("swerve_steering_controller.swerve_steering_controller");
using CallbackReturn = controller_interface::CallbackReturn;
using namespace std::chrono_literals;
class SwerveSteeringController : public controller_interface::ControllerInterface
{
public:
  SWERVE_STEERING_CONTROLLER_PUBLIC
  SwerveSteeringController();

  // controller_interface::return_type init(
  //   const std::string & controller_name,
  //   rclcpp::Node::SharedPtr & root_node,
  //   rclcpp::Node::SharedPtr & controller_node
  // ) override;

  SWERVE_STEERING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SWERVE_STEERING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SWERVE_STEERING_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SWERVE_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  SWERVE_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;


private:
  // NEW
  // Handle structure for the swerve module interfaces
  struct SwerveModuleHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command;
  };

  std::vector<SwerveModuleHandle> swerve_handles_;

  // std::vector<SwerveModuleHandle> front_left_wheel_handles;
  // std::vector<SwerveModuleHandle> front_right_wheel_handles;
  // std::vector<SwerveModuleHandle> rear_left_wheel_handles;
  // std::vector<SwerveModuleHandle> rear_right_wheel_handles;

  // // parameters from ROS for the swerve_steering_controller
  // std::shared_ptr<ParamListener> param_listener_;
  // Params params_;

  // OLD

  std::string name_;

  std::string base_frame_id_;
  std::string odom_frame_id_;

  rclcpp::Duration publish_period_;
  rclcpp::Time last_state_publish_time_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> tf_odom_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Point>>
    avg_intersection_publisher_ = nullptr;

  Odometry odometry_;

  double infinity_tol_;
  double intersection_tol_;

  /// Speed limiters:
  utils::command last1_cmd_;
  utils::command last0_cmd_;
  SpeedLimiter limiter_lin_x_;
  SpeedLimiter limiter_lin_y_;
  SpeedLimiter limiter_ang_;

  /// Previous time and velocities from the encoders:
  rclcpp::Time time_previous_;
  std::vector<double> wheels_velocities_previous_;
  std::vector<double> holders_velocities_previous_;

  std::vector<double> wheels_desired_velocities_previous_;
  std::vector<double> holders_desired_velocities_previous_;
  std::vector<double> holders_desired_positions_previous_;

  bool enable_odom_tf_;
  bool publish_wheel_joint_controller_state_;

  size_t wheel_joints_size_;

  std::vector<wheel> wheels_;
  std::vector<hardware_interface::JointHandle> wheels_joints_handles_;
  std::vector<hardware_interface::JointHandle> holders_joints_handles_;

  utils::command cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
  realtime_tools::RealtimeBuffer<utils::command> commands_buffer_;

  std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState> >
    controller_state_pub_;

  void cmd_callback(const geometry_msgs::msg::Twist & command);

  bool getWheelParams(
    const std::string & wheel_param,
    const std::string & holder_param,
    std::vector<std::string> & wheel_names,
    std::vector<std::string> & holder_names);
  bool getXmlStringList(
    const std::string & list_param,
    std::vector<std::string> & returned_names);

  void setOdomPubFields();

  // Controller state publisher.. includes the whels and the holders
  std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState> >
    controller_state_publisher;

  void set_to_initial_state();

  void publishWheelData(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    std::vector<double> wheels_desired_velocities, std::vector<double> holders_desired_positions);
};
}  // namespace swerve_steering_controller
