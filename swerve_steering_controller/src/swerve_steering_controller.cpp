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

#include <pluginlib/class_list_macros.hpp>
#include <swerve_steering_controller/swerve_steering_controller.hpp>

namespace swerve_steering_controller
{
SwerveSteeringController::SwerveSteeringController()
: cmd_(),
  base_frame_id_("base_link"),
  odom_frame_id_("odom"),
  enable_odom_tf_(true),
  wheel_joints_size_(0),
  publish_wheel_joint_controller_state_(false)
{
}

CallbackReturn SwerveSteeringController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto velocity_interfaces = get_node()->get_command_interfaces("velocity");
  auto position_interfaces = get_node()->get_command_interfaces("position");

  for (const auto & wheel_name : wheels_names)
  {
    for (auto & interface : velocity_interfaces)
    {
      if (interface.get_name() == wheel_name)
      {
        // Save the velocity interface for this wheel
        wheels_joints_handles_.emplace_back(interface);
        break;
      }
    }

    // Similar loop for position interfaces and holders_joints_handles_
  }
}

// bool SwerveSteeringController::init(
//   hardware_interface::RobotHW * robot_hw, rclcpp::NodeHandle & root_nh,
//   rclcpp::NodeHandle & controller_nh)
CallbackReturn SwerveSteeringController::on_init()
{
  const std::string complete_ns = get_node().getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  // name_ = complete_ns.substr(id + 1);

  std::vector<std::string> holders_names, wheels_names;
  if (!getWheelParams(get_node(), "wheels", "holders", wheels_names, holders_names))
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "Couldn't import the wheels");
    return false;
  }
  std::vector<double> radii;
  std::vector<std::array<double, 2>> positions;
  for (const auto & it : wheels_)
  {
    radii.push_back(it.radius);
    positions.push_back(it.position);
  }
  odometry_.setWheelsParams(radii, positions);

  wheel_joints_size_ = wheels_names.size();

  wheels_joints_handles_.resize(wheel_joints_size_);
  holders_joints_handles_.resize(wheel_joints_size_);

  get_node().param(
    "publish_wheel_joint_controller_state", publish_wheel_joint_controller_state_,
    publish_wheel_joint_controller_state_);

  // Velocity and acceleration limits:
  get_node().param(
    "linear/x/has_velocity_limits", limiter_lin_x_.has_velocity_limits,
    limiter_lin_x_.has_velocity_limits);
  get_node().param(
    "linear/x/has_acceleration_limits", limiter_lin_x_.has_acceleration_limits,
    limiter_lin_x_.has_acceleration_limits);
  get_node().param(
    "linear/x/max_velocity", limiter_lin_x_.max_velocity, limiter_lin_x_.max_velocity);
  get_node().param(
    "linear/x/min_velocity", limiter_lin_x_.min_velocity, -limiter_lin_x_.max_velocity);
  get_node().param(
    "linear/x/max_acceleration", limiter_lin_x_.max_acceleration, limiter_lin_x_.max_acceleration);
  get_node().param(
    "linear/x/min_acceleration", limiter_lin_x_.min_acceleration, -limiter_lin_x_.max_acceleration);

  get_node().param(
    "linear/y/has_velocity_limits", limiter_lin_y_.has_velocity_limits,
    limiter_lin_y_.has_velocity_limits);
  get_node().param(
    "linear/y/has_acceleration_limits", limiter_lin_y_.has_acceleration_limits,
    limiter_lin_y_.has_acceleration_limits);
  get_node().param(
    "linear/y/max_velocity", limiter_lin_y_.max_velocity, limiter_lin_y_.max_velocity);
  get_node().param(
    "linear/y/min_velocity", limiter_lin_y_.min_velocity, -limiter_lin_y_.max_velocity);
  get_node().param(
    "linear/y/max_acceleration", limiter_lin_y_.max_acceleration, limiter_lin_y_.max_acceleration);
  get_node().param(
    "linear/y/min_acceleration", limiter_lin_y_.min_acceleration, -limiter_lin_y_.max_acceleration);

  get_node().param(
    "angular/z/has_velocity_limits", limiter_ang_.has_velocity_limits,
    limiter_ang_.has_velocity_limits);
  get_node().param(
    "angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits,
    limiter_ang_.has_acceleration_limits);
  get_node().param("angular/z/max_velocity", limiter_ang_.max_velocity, limiter_ang_.max_velocity);
  get_node().param("angular/z/min_velocity", limiter_ang_.min_velocity, -limiter_ang_.max_velocity);
  get_node().param(
    "angular/z/max_acceleration", limiter_ang_.max_acceleration, limiter_ang_.max_acceleration);
  get_node().param(
    "angular/z/min_acceleration", limiter_ang_.min_acceleration, -limiter_ang_.max_acceleration);

  // Wheel joint controller state:
  if (publish_wheel_joint_controller_state_)
  {
    auto publish_wheel_joint_controller_state_standard =
      get_node()->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
        "wheel_joint_controller_state", 100);
    controller_state_pub_.reset(
      new realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>(
        publish_wheel_joint_controller_state_standard));

    const size_t num_joints = wheel_joints_size_ * 2;  // for the steering joint and the wheel joint

    controller_state_pub_->msg_.joint_names.resize(num_joints);

    controller_state_pub_->msg_.desired.positions.resize(num_joints);
    controller_state_pub_->msg_.desired.velocities.resize(num_joints);
    controller_state_pub_->msg_.desired.accelerations.resize(num_joints);
    controller_state_pub_->msg_.desired.effort.resize(num_joints);

    controller_state_pub_->msg_.actual.positions.resize(num_joints);
    controller_state_pub_->msg_.actual.velocities.resize(num_joints);
    controller_state_pub_->msg_.actual.accelerations.resize(num_joints);
    controller_state_pub_->msg_.actual.effort.resize(num_joints);

    controller_state_pub_->msg_.error.positions.resize(num_joints);
    controller_state_pub_->msg_.error.velocities.resize(num_joints);
    controller_state_pub_->msg_.error.accelerations.resize(num_joints);
    controller_state_pub_->msg_.error.effort.resize(num_joints);

    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      controller_state_pub_->msg_.joint_names[i] = wheels_names[i];
      controller_state_pub_->msg_.joint_names[i + wheel_joints_size_] = holders_names[i];
    }
  }

  // remember: set the odom fields when it all works
  setOdomPubFields();

  hardware_interface::VelocityJointInterface * const vel_joint_hw =
    robot_hw->get<hardware_interface::VelocityJointInterface>();
  hardware_interface::PositionJointInterface * const pos_joint_hw =
    robot_hw->get<hardware_interface::PositionJointInterface>();

  // Get the joint object to use in the realtime loop
  for (size_t i = 0; i < wheel_joints_size_; ++i)
  {
    RCLCPP_INFO_STREAM_ONCE(
      LOGGER, "Adding a wheel with joint name: "
                << wheels_names[i] << " and a holder with joint name: " << holders_names[i]);
    wheels_joints_handles_[i] = vel_joint_hw->getHandle(wheels_names[i]);    // throws on failure
    holders_joints_handles_[i] = pos_joint_hw->getHandle(holders_names[i]);  // throws on failure
  }

  // Odometry related:
  double publish_rate;
  get_node().param("publish_rate", publish_rate, 50.0);
  RCLCPP_INFO_STREAM_ONCE(
    LOGGER, "Controller state will be published at " << publish_rate << "Hz.");
  publish_period_ = rclcpp::Duration(1.0 / publish_rate);

  int velocity_rolling_window_size = 4;
  get_node().param(
    "velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
  RCLCPP_INFO_STREAM_ONCE(
    LOGGER, "Velocity rolling window size of " << velocity_rolling_window_size << ".");

  odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

  get_node().param("base_frame_id", base_frame_id_, base_frame_id_);
  RCLCPP_INFO_STREAM_ONCE(LOGGER, "Base frame_id set to " << base_frame_id_);

  get_node().param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
  RCLCPP_INFO_STREAM_ONCE(
    LOGGER, "Publishing to tf is " << (enable_odom_tf_ ? "enabled" : "disabled"));

  get_node().param("infinity_tolerance", infinity_tol_, 1000.0);
  RCLCPP_INFO_STREAM_ONCE(LOGGER, "infinity tolerance set to " << infinity_tol_);

  get_node().param("intersection_tolerance", intersection_tol_, 0.1);
  RCLCPP_INFO_STREAM_ONCE(LOGGER, "intersection tolerance set to " << intersection_tol_);

  // cmd_subscriber_ =
  //   get_node().subscribe("cmd_vel", 1, &SwerveSteeringController::cmd_callback, this);
  cmd_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&SwerveSteeringController::cmd_callback, this, std::placeholders::_1));

  return true;
}

controller_interface::return_type SwerveSteeringController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::vector<double> wheels_omega, holders_theta;
  std::vector<int> directions;
  for (size_t i = 0; i < wheels_joints_handles_.size(); ++i)
  {
    wheels_omega.push_back(wheels_joints_handles_[i].getVelocity());
    double angle = holders_joints_handles_[i].getPosition();
    holders_theta.push_back(angle);
    wheels_[i].set_current_angle(angle);  // to keep the wheel object updated
    directions.push_back(wheels_[i].get_omega_direction());
  }
  std::array<double, 2> intersection_point = {0, 0};
  odometry_.update(wheels_omega, holders_theta, directions, time, &intersection_point);
  if (avg_intersection_publisher_->trylock())
  {
    avg_intersection_publisher_->msg_.x = intersection_point[0];
    avg_intersection_publisher_->msg_.y = intersection_point[1];

    avg_intersection_publisher_->unlockAndPublish();
  }
  // Publish odometry message
  if (last_state_publish_time_ + publish_period_ < time)
  {
    last_state_publish_time_ += publish_period_;

    // Compute and store orientation info
    const geometry_msgs::msg::Quaternion orientation(
      tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

    // Populate odom message and publish
    if (odom_publisher_->trylock())
    {
      odom_publisher_->msg_.header.stamp = time;
      odom_publisher_->msg_.pose.pose.position.x = odometry_.getX();
      odom_publisher_->msg_.pose.pose.position.y = odometry_.getY();
      odom_publisher_->msg_.pose.pose.orientation = orientation;
      odom_publisher_->msg_.twist.twist.linear.x = odometry_.getLinearX();
      odom_publisher_->msg_.twist.twist.linear.y = odometry_.getLinearY();
      odom_publisher_->msg_.twist.twist.angular.z = odometry_.getAngular();
      odom_publisher_->unlockAndPublish();
    }

    // Publish tf /odom frame
    if (enable_odom_tf_ && tf_odom_publisher_->trylock())
    {
      geometry_msgs::msg::TransformStamped & odom_frame = tf_odom_publisher_->msg_.transforms[0];
      odom_frame.header.stamp = time;
      odom_frame.transform.translation.x = odometry_.getX();
      odom_frame.transform.translation.y = odometry_.getY();
      odom_frame.transform.rotation = orientation;
      tf_odom_publisher_->unlockAndPublish();
    }
  }

  // MOVE ROBOT
  // Retrieve current velocity command and time step
  utils::command current_cmd = *(commands_buffer_.readFromRT());

  // Limit velocities and accelerations:
  const double cmd_dt(period.toSec());

  limiter_lin_x_.limit(current_cmd.x, last0_cmd_.x, last1_cmd_.x, cmd_dt);
  limiter_lin_y_.limit(current_cmd.y, last0_cmd_.y, last1_cmd_.y, cmd_dt);
  limiter_ang_.limit(current_cmd.w, last0_cmd_.w, last1_cmd_.w, cmd_dt);

  last1_cmd_ = last0_cmd_;
  last0_cmd_ = current_cmd;

  // Compute wheels velocities and set wheels velocities
  std::vector<double> desired_velocities, desired_positions;

  for (size_t i = 0; i < wheel_joints_size_; ++i)
  {
    // double w = -1*current_cmd.w; //and will negate the signals of w in the expressions below

    double wheel_vx = current_cmd.x - current_cmd.w * wheels_[i].position[1] -
                      wheels_[i].offset * cos(holders_joints_handles_[i].getPosition());
    double wheel_vy = current_cmd.y + current_cmd.w * wheels_[i].position[0] +
                      wheels_[i].offset * sin(holders_joints_handles_[i].getPosition());

    // get the required wheel omega and the required wheel steering angle
    double w_w = sqrt(pow(wheel_vx, 2) + pow(wheel_vy, 2)) / wheels_[i].radius;
    double w_th = atan2(wheel_vy, wheel_vx);

    // process the requested w,th in the wheel class
    wheels_[i].set_current_angle(
      holders_joints_handles_[i].getPosition());  // to keep the wheel object updated

    wheels_[i].set_command_velocity(w_w);
    wheels_[i].set_command_angle(
      w_th);  // this will do the closest angle calculation and set it in the object.

    // get the actual w,th to be applied on the wheels.
    double w_applied = wheels_[i].get_command_velocity();
    double th_applied = wheels_[i].get_command_angle();

    if (publish_wheel_joint_controller_state_)
    {
      desired_velocities.push_back(w_applied);
      desired_positions.push_back(th_applied);
    }

    wheels_joints_handles_[i].setCommand(w_applied);
    holders_joints_handles_[i].setCommand(th_applied);
  }

  publishWheelData(time, period, desired_velocities, desired_positions);

  time_previous_ = time;

  return controller_interface::return_type::OK;
}

CallbackReturn SwerveSteeringController::on_activate(const rclcpp::Time & time)
{
  set_to_initial_state();

  odometry_.init(time, infinity_tol_, intersection_tol_);

  last_state_publish_time_ = time;
  time_previous_ = time;

  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveSteeringController::on_deactive(const rclcpp::Time & time)
{
  set_to_initial_state();

  return CallbackReturn::SUCCESS;
}

void SwerveSteeringController::set_to_initial_state()
{
  const double vel = 0.0;
  for (size_t i = 0; i < wheel_joints_size_; ++i)
  {
    wheels_joints_handles_[i].setCommand(0);
    holders_joints_handles_[i].setCommand(0);
  }
}

void SwerveSteeringController::cmd_callback(const geometry_msgs::msg::Twist & command)
{
  if (isRunning())
  {
    if (
      !std::isfinite(command.angular.z) || !std::isfinite(command.linear.x) ||
      !std::isfinite(command.linear.y))
    {
      RCLCPP_WARN_THROTTLE(LOGGER, CLOCK, 1000, "Received NaN in velocity command. Ignoring.");
      return;
    }

    cmd_.w = command.angular.z;
    cmd_.x = command.linear.x;
    cmd_.y = command.linear.y;
    cmd_.stamp = rclcpp::Time::now();
    commands_buffer_.writeFromNonRT(cmd_);
    RCLCPP_DEBUG_STREAM_ONCE(
      LOGGER, "Added values to command. "
                << "Ang: " << cmd_.w << ", "
                << "Lin x,y: (" << cmd_.x << " , " << cmd_.y << "), "
                << "Stamp: " << cmd_.stamp.nanoseconds() / 1000000000 << " s");
  }
  else
  {
    RCLCPP_ERROR_ONCE(LOGGER, "Can't accept new commands. Controller is not running.");
  }
}

bool SwerveSteeringController::getWheelParams(
  const std::string & wheel_param, const std::string & holder_param,
  std::vector<std::string> & wheel_names, std::vector<std::string> & holder_names)
{
  // prepare names for the joint handlers
  if (!(getXmlStringList(get_node(), wheel_param, wheel_names) &&
        getXmlStringList(get_node(), holder_param, holder_names)))
  {
    return false;
  }

  /*prepare radii, limits, limitless flag for the wheels classes*/

  // initialize wheels vector
  wheels_.resize(wheel_names.size(), wheel());

  // radius
  XmlRpc::XmlRpcValue xml_list;
  if (!get_node().getParam("radii", xml_list))
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "Couldn't retrieve list param 'radii'");
    return false;
  }

  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "list param 'radii' is not of type array");
    return false;
  }

  if (xml_list.size() == 0)
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param 'radii' is an empty list");
    return false;
  }

  for (int i = 0; i < xml_list.size(); ++i)
  {
    if (
      xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
      xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      RCLCPP_ERROR_STREAM_ONCE(
        LOGGER, "List param 'radii' #" << i << " isn't of type double or int");
      return false;
    }
    wheels_[i].radius = static_cast<double>(xml_list[i]);
  }

  // position
  if (!get_node().getParam("positions", xml_list))
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "Couldn't retrieve list param 'position'");
    return false;
  }

  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "list param 'position' is not of type array");
    return false;
  }
  if (xml_list.size() == 0)
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param 'position' is an empty list");
    return false;
  }

  else if (xml_list.size() != wheel_names.size())
  {
    RCLCPP_ERROR_STREAM_ONCE(
      LOGGER, "List param 'position' size is not equal to List param 'wheel' size");
    return false;
  }

  for (int i = 0; i < xml_list.size(); ++i)
  {
    if (xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param 'position' #" << i << " isn't of type array");
      return false;
    }
    if (xml_list[i].size() != 2)
    {
      RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param 'position' #" << i << " size isn't 2");
      return false;
    }

    std::array<double, 2> position;
    for (int j = 0; j < 2; ++j)
    {
      if (
        xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeInt &&
        xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      {
        RCLCPP_ERROR_STREAM_ONCE(
          LOGGER, "List param 'position' #" << i << " #" << j << " isn't of type int or double");
        return false;
      }
      position[j] = utils::theta_map(static_cast<double>(xml_list[i][j]));
    }

    wheels_[i].set_position(position);
  }

  // limitless flag
  if (!get_node().getParam("limitless", xml_list))
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "Couldn't retrieve list param 'limitless'");
    return false;
  }

  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "list param 'limitless' is not of type array");
    return false;
  }

  if (xml_list.size() == 0)
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param 'limitless' is an empty list");
    return false;
  }

  for (int i = 0; i < xml_list.size(); ++i)
  {
    if (xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    {
      RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param 'limitless' #" << i << " isn't of type bool");
      return false;
    }
    wheels_[i].set_limitless(static_cast<bool>(xml_list[i]));
  }

  // limits
  if (!get_node().getParam("limits", xml_list))
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "Couldn't retrieve list param 'limits'");
    return false;
  }

  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "list param 'limits' is not of type array");
    return false;
  }
  if (xml_list.size() == 0)
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param 'limits' is an empty list");
    return false;
  }

  else if (xml_list.size() != wheel_names.size())
  {
    RCLCPP_ERROR_STREAM_ONCE(
      LOGGER, "List param 'limits' size is not equal to List param 'wheel' size");
    return false;
  }

  for (int i = 0; i < xml_list.size(); ++i)
  {
    // if the wheel is limitless, no need to read the limits
    if (wheels_[i].get_limitless())
    {
      RCLCPP_INFO_STREAM_ONCE(
        LOGGER, "List param 'limits' #" << i << " skipped because limitless flag is set to true");
      continue;
    }

    if (xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param 'limits' #" << i << " isn't of type array");
      return false;
    }
    if (xml_list[i].size() != 2)
    {
      RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param 'limits' #" << i << " size isn't 2");
      return false;
    }

    std::array<double, 2> limit;
    for (int j = 0; j < 2; ++j)
    {
      if (
        xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeInt &&
        xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      {
        RCLCPP_ERROR_STREAM_ONCE(
          LOGGER, "List param 'limits' #" << i << " #" << j << " isn't of type int or double");
        return false;
      }
      limit[j] = utils::theta_map(static_cast<double>(xml_list[i][j]));
    }

    wheels_[i].set_rotation_limits(limit);
  }

  // offsets
  if (!get_node().getParam("offsets", xml_list))
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "Couldn't retrieve list param 'offsets'");
    return false;
  }

  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "list param 'offsets' is not of type array");
    return false;
  }

  if (xml_list.size() == 0)
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param 'offsets' is an empty list");
    return false;
  }

  for (int i = 0; i < xml_list.size(); ++i)
  {
    if (
      xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
      xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      RCLCPP_ERROR_STREAM_ONCE(
        LOGGER, "List param 'offsets' #" << i << " isn't of type int or double ");
      return false;
    }
    wheels_[i].offset = static_cast<double>(xml_list[i]);
  }

  return true;
}

bool SwerveSteeringController::getXmlStringList(
  const std::string & list_param, std::vector<std::string> & returned_names)
{
  XmlRpc::XmlRpcValue xml_list;
  if (!get_node().getParam(list_param, xml_list))
  {
    RCLCPP_ERROR_STREAM_ONCE(LOGGER, "Couldn't retrieve list param '" << list_param << "'.");
    return false;
  }

  if (xml_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (xml_list.size() == 0)
    {
      RCLCPP_ERROR_STREAM_ONCE(LOGGER, "List param '" << list_param << "' is an empty list");
      return false;
    }

    for (int i = 0; i < xml_list.size(); ++i)
    {
      if (xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        RCLCPP_ERROR_STREAM_ONCE(
          LOGGER, "List param '" << list_param << "' #" << i << " isn't a string.");
        return false;
      }
      else
      {
        returned_names.push_back(static_cast<std::string>(xml_list[i]));
      }
    }
  }

  else if (xml_list.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    returned_names.push_back(xml_list);
  }

  else
  {
    RCLCPP_ERROR_STREAM_ONCE(
      LOGGER, "List param '" << list_param << "' is neither a list of strings nor a string.");
    return false;
  }

  return true;
}

void SwerveSteeringController::setOdomPubFields()
{
  auto avg_intersection_publisher_standard =
    get_node()->create_publisher<geometry_msgs::msg::Point>("avg_intersection", 100);
  avg_intersection_publisher_.reset(
    new realtime_tools::RealtimePublisher<geometry_msgs::msg::Point>(
      avg_intersection_publisher_standard));  // to show the avg intersection

  // Get and check params for covariances
  XmlRpc::XmlRpcValue pose_cov_list;
  get_node().getParam("pose_covariance_diagonal", pose_cov_list);
  ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pose_cov_list.size() == 6);
  for (int i = 0; i < pose_cov_list.size(); ++i)
    ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  XmlRpc::XmlRpcValue twist_cov_list;
  get_node().getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // Setup odometry realtime publisher + odom message constant fields
  auto odom_publisher_standard = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
  odom_publisher_.reset(
    new realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>(odom_publisher_standard));
  odom_publisher_->msg_.header.frame_id = odom_frame_id_;
  odom_publisher_->msg_.child_frame_id = base_frame_id_;
  odom_publisher_->msg_.pose.pose.position.z = 0;
  odom_publisher_->msg_.pose.covariance = {
    static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0., 0.,
    static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0., 0., 0.,
    static_cast<double>(pose_cov_list[2]), 0., 0., 0., 0., 0., 0.,
    static_cast<double>(pose_cov_list[3]), 0., 0., 0., 0., 0., 0.,
    static_cast<double>(pose_cov_list[4]), 0., 0., 0., 0., 0., 0.,
    static_cast<double>(pose_cov_list[5])};
  odom_publisher_->msg_.twist.twist.linear.y = 0;
  odom_publisher_->msg_.twist.twist.linear.z = 0;
  odom_publisher_->msg_.twist.twist.angular.x = 0;
  odom_publisher_->msg_.twist.twist.angular.y = 0;
  odom_publisher_->msg_.twist.covariance = {
    static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0., 0.,
    static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0., 0., 0.,
    static_cast<double>(twist_cov_list[2]), 0., 0., 0., 0., 0., 0.,
    static_cast<double>(twist_cov_list[3]), 0., 0., 0., 0., 0., 0.,
    static_cast<double>(twist_cov_list[4]), 0., 0., 0., 0., 0., 0.,
    static_cast<double>(twist_cov_list[5])};

  auto tf_odom_publisher_standard =
    get_node()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
  tf_odom_publisher_.reset(
    new realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>(tf_odom_publisher_standard));
  tf_odom_publisher_->msg_.transforms.resize(1);
  tf_odom_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_publisher_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_publisher_->msg_.transforms[0].header.frame_id = odom_frame_id_;
}

void SwerveSteeringController::publishWheelData(
  const rclcpp::Time & time, const rclcpp::Duration & period,
  std::vector<double> wheels_desired_velocities, std::vector<double> holders_desired_positions)
{
  if (publish_wheel_joint_controller_state_ && controller_state_pub_->trylock())
  {
    const double cmd_dt(period.seconds());

    // Compute desired wheels velocities, that is before applying limits:
    controller_state_pub_->msg_.header.stamp = time;
    const double control_duration = (time - time_previous_).seconds();

    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      double holder_desired_velocity =
        (holders_desired_positions[i] - holders_desired_positions_previous_[i]) / cmd_dt;

      const double wheel_acc =
        (wheels_joints_handles_[i].getVelocity() - wheels_velocities_previous_[i]) /
        control_duration;
      const double holder_acc =
        (holders_joints_handles_[i].getVelocity() - holders_velocities_previous_[i]) /
        control_duration;

      // Actual
      controller_state_pub_->msg_.actual.positions[i] = wheels_joints_handles_[i].getPosition();
      controller_state_pub_->msg_.actual.velocities[i] = wheels_joints_handles_[i].getVelocity();
      controller_state_pub_->msg_.actual.accelerations[i] = wheel_acc;
      controller_state_pub_->msg_.actual.effort[i] = wheels_joints_handles_[i].getEffort();

      controller_state_pub_->msg_.actual.positions[i + wheel_joints_size_] =
        holders_joints_handles_[i].getPosition();
      controller_state_pub_->msg_.actual.velocities[i + wheel_joints_size_] =
        holders_joints_handles_[i].getVelocity();
      controller_state_pub_->msg_.actual.accelerations[i + wheel_joints_size_] = holder_acc;
      controller_state_pub_->msg_.actual.effort[i + wheel_joints_size_] =
        holders_joints_handles_[i].getEffort();

      // Desired
      controller_state_pub_->msg_.desired.positions[i] += wheels_desired_velocities[i] * cmd_dt;
      controller_state_pub_->msg_.desired.velocities[i] = wheels_desired_velocities[i];
      controller_state_pub_->msg_.desired.accelerations[i] =
        (wheels_desired_velocities[i] - holders_desired_velocities_previous_[i]) * cmd_dt;
      controller_state_pub_->msg_.desired.effort[i] = std::numeric_limits<double>::quiet_NaN();

      controller_state_pub_->msg_.desired.positions[i + wheel_joints_size_] +=
        holder_desired_velocity * cmd_dt;
      controller_state_pub_->msg_.desired.velocities[i + wheel_joints_size_] =
        holder_desired_velocity;
      controller_state_pub_->msg_.desired.accelerations[i + wheel_joints_size_] =
        (holder_desired_velocity - holders_desired_velocities_previous_[i]) * cmd_dt;
      controller_state_pub_->msg_.desired.effort[i + wheel_joints_size_] =
        std::numeric_limits<double>::quiet_NaN();

      // Error
      controller_state_pub_->msg_.error.positions[i] =
        controller_state_pub_->msg_.desired.positions[i] -
        controller_state_pub_->msg_.actual.positions[i];
      controller_state_pub_->msg_.error.velocities[i] =
        controller_state_pub_->msg_.desired.velocities[i] -
        controller_state_pub_->msg_.actual.velocities[i];
      controller_state_pub_->msg_.error.accelerations[i] =
        controller_state_pub_->msg_.desired.accelerations[i] -
        controller_state_pub_->msg_.actual.accelerations[i];
      controller_state_pub_->msg_.error.effort[i] = controller_state_pub_->msg_.desired.effort[i] -
                                                    controller_state_pub_->msg_.actual.effort[i];

      controller_state_pub_->msg_.error.positions[i + wheel_joints_size_] =
        controller_state_pub_->msg_.desired.positions[i + wheel_joints_size_] -
        controller_state_pub_->msg_.actual.positions[i + wheel_joints_size_];
      controller_state_pub_->msg_.error.velocities[i + wheel_joints_size_] =
        controller_state_pub_->msg_.desired.velocities[i + wheel_joints_size_] -
        controller_state_pub_->msg_.actual.velocities[i + wheel_joints_size_];
      controller_state_pub_->msg_.error.accelerations[i + wheel_joints_size_] =
        controller_state_pub_->msg_.desired.accelerations[i + wheel_joints_size_] -
        controller_state_pub_->msg_.actual.accelerations[i + wheel_joints_size_];
      controller_state_pub_->msg_.error.effort[i + wheel_joints_size_] =
        controller_state_pub_->msg_.desired.effort[i + wheel_joints_size_] -
        controller_state_pub_->msg_.actual.effort[i + wheel_joints_size_];

      // Save previous velocities to compute acceleration
      wheels_velocities_previous_[i] = wheels_joints_handles_[i].getVelocity();
      wheels_desired_velocities_previous_[i] = wheels_desired_velocities[i];

      holders_velocities_previous_[i] = holders_joints_handles_[i].getVelocity();
      holders_desired_positions_previous_[i] = holders_desired_positions[i];
      holders_desired_velocities_previous_[i] = holder_desired_velocity;
    }

    controller_state_pub_->unlockAndPublish();
  }
}

}  // namespace swerve_steering_controller
PLUGINLIB_EXPORT_CLASS(
  swerve_steering_controller::SwerveSteeringController, controller_interface::ControllerInterface)
