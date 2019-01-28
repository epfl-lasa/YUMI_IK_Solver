/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>

#include <industrial_msgs/RobotStatus.h>

#include "abb_egm_driver/abb_publisher_handler.h"

namespace abb
{
namespace publisher_handler
{

// TODO: Fix transform (reuse ROS-I abb_driver's abb::utils).
void PublisherHandler::init(std::vector<std::string>& joint_names)
{
  pub_joint_control_state_ = node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
  pub_joint_sensor_state_  = node_.advertise<sensor_msgs::JointState>("joint_states", 1);
  pub_robot_status_        = node_.advertise<industrial_msgs::RobotStatus>("robot_status", 1);

  // Save "complete" joint-name list, preserving any blank entries for later use.
  this->all_joint_names_ = joint_names;
}

// TODO: Add input for robot status and publish it as well.
bool PublisherHandler::publishInformation(const std::vector<double>& all_joint_pos)
{
  control_msgs::FollowJointTrajectoryFeedback control_state;
  sensor_msgs::JointState sensor_state;
  bool rtn = true;

  if (createMessages(all_joint_pos, &control_state, &sensor_state))
  {
    this->pub_joint_control_state_.publish(control_state);
    this->pub_joint_sensor_state_.publish(sensor_state);
  }
  else
  {
    rtn = false;
  }

  return rtn;
}

bool PublisherHandler::createMessages(const std::vector<double>& all_joint_pos,
                                      control_msgs::FollowJointTrajectoryFeedback* control_state,
                                      sensor_msgs::JointState* sensor_state)
{
  // Apply transform to joint positions, if required.
  std::vector<double> xform_joint_pos;
  if (!transform(all_joint_pos, &xform_joint_pos))
  {
    ROS_ERROR("Failed to transform joint positions");
    return false;
  }

  // Select specific joints for publishing.
  std::vector<double> pub_joint_pos;
  std::vector<std::string> pub_joint_names;
  if (!select(xform_joint_pos, all_joint_names_, &pub_joint_pos, &pub_joint_names))
  {
    ROS_ERROR("Failed to select joints for publishing");
    return false;
  }

  // Assign values to messages.
  control_msgs::FollowJointTrajectoryFeedback tmp_control_state;  // Always start with a "clean" message.
  tmp_control_state.header.stamp = ros::Time::now();
  tmp_control_state.joint_names = pub_joint_names;
  tmp_control_state.actual.positions = pub_joint_pos;
  *control_state = tmp_control_state;

  sensor_msgs::JointState tmp_sensor_state;
  tmp_sensor_state.header.stamp = ros::Time::now();
  tmp_sensor_state.name = pub_joint_names;
  tmp_sensor_state.position = pub_joint_pos;
  *sensor_state = tmp_sensor_state;

  return true;
}

bool PublisherHandler::select(const std::vector<double>& all_joint_pos,
                              const std::vector<std::string>& all_joint_names,
                              std::vector<double>* pub_joint_pos,
                              std::vector<std::string>* pub_joint_names)
{
  ROS_ASSERT(all_joint_pos.size() == all_joint_names.size());

  pub_joint_pos->clear();
  pub_joint_names->clear();

  // Skip over "blank" joint names.
  for (int i=0; i<all_joint_pos.size(); ++i)
  {
    if (all_joint_names[i].empty())
      continue;

    pub_joint_pos->push_back(all_joint_pos[i]);
    pub_joint_names->push_back(all_joint_names[i]);
  }

  return true;
}

} // publisher_handler
} // abb
