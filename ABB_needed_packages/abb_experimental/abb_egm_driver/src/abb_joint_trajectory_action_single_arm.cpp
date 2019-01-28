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

#include <industrial_utils/utils.h>

#include "abb_egm_driver/abb_joint_trajectory_actions.h"

using namespace abb::egm_interface;
using namespace abb::egm_interface::egm_common_values;
using namespace abb::rws_interface;

namespace abb
{
namespace joint_trajectory_action
{
/*
 * Class definitions: JointTrajectoryActionSingleArm
 */

/*
 * Primary methods
 */

JointTrajectoryActionSingleArm::JointTrajectoryActionSingleArm(boost::asio::io_service &io_service,
                                                               std::string robot_ip_address,
                                                               std::string rws_port)
  :
  action_server_(node_,
                 "joint_trajectory_action",
                 boost::bind(&JointTrajectoryActionSingleArm::goalCB, this, _1),
                 boost::bind(&JointTrajectoryActionSingleArm::cancelCB, this, _1),
                 false),
  egm_interface_(io_service, egm_common_values::communication::DEFAULT_PORT_NUMBER),
  task_name_(rws_strings::system_defined::rapid::task_names::RAPID_TASK_ROB1)
{
  p_rws_interface_.reset(new RWSInterface(robot_ip_address, rws_port));

  EGMInterfaceConfiguration configuration;
  configuration.basic.use_conditions = false;
  configuration.communication.use_speed = true;
  configuration.simple_interpolation.use_speed = true;
  configuration.simple_interpolation.use_acceleration = true;
  configuration.simple_interpolation.use_interpolation = true;
  egm_interface_.setConfiguration(configuration);

  action_server_.start();
}

void JointTrajectoryActionSingleArm::goalCB(JointTrajectoryActionServer::GoalHandle & gh)
{
  ROS_INFO("Received new goal");

  // Reject all goals as long as the Robot Web Services (RWS) connection is not ready.
  if (!rws_connection_ready_)
  {
    ROS_ERROR("Joint trajectory action rejected: waiting for Robot Web Services (RWS) "
              "connection to be ready (robot controllern need to be in AUTO mode and have RAPID running)");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt,
                   "Waiting for Robot Web Services (RWS) connection to be ready "
                   "(robot controllern need to be in AUTO mode and have RAPID running)");

    // No point in continuing: already rejected.
    return;
  }

  // Reject all goals as long as we haven't heard from the remote controller.
  if (!trajectory_state_recvd_)
  {
    ROS_ERROR("Joint trajectory action rejected: waiting for (initial) feedback from controller");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Waiting for (initial) feedback from controller");

    // No point in continuing: already rejected.
    return;
  }

  if(!p_rws_interface_->isRobotIdle(task_name_))
  {
    ROS_ERROR("Joint trajectory action rejected: Robot controller is not ready to recieve a command");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Robot controller is not ready to recieve a command");

    // No point in continuing: already rejected.
    return;
  }

  if (!gh.getGoal()->trajectory.points.empty())
  {
    if (industrial_utils::isSimilar(joint_names_, gh.getGoal()->trajectory.joint_names))
    {
      // Cancels the currently active goal.
      if (has_active_goal_)
      {
        ROS_WARN("Received new goal, canceling current goal");
        abortGoal();
      }

      // Sends the trajectory along to the controller.
      if (withinGoalConstraints(last_trajectory_state_, gh.getGoal()->trajectory))
      {
        ROS_INFO_STREAM("Already within goal constraints, setting goal succeeded");
        gh.setAccepted();
        gh.setSucceeded();
        has_active_goal_ = false;
      }
      else
      {
        gh.setAccepted();
        active_goal_ = gh;
        has_active_goal_ = true;

        ROS_INFO("Publishing trajectory");

        current_traj_ = active_goal_.getGoal()->trajectory;

        control_msgs::FollowJointTrajectoryGoal goal = *active_goal_.getGoal();

        egm_interface_.addTrajectory(goal, true);

        startEGM();
      }
    }
    else
    {
      ROS_ERROR("Joint trajectory action failing on invalid joints");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(rslt, "Joint names do not match");
    }
  }
  else
  {
    ROS_ERROR("Joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Empty trajectory");
  }

  // Adding some informational log messages to indicate unsupported goal constraints.
  if (gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
  {
    ROS_WARN_STREAM("Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM("Ignoring goal tolerance in action, using paramater tolerance of " <<
                    goal_threshold_ << " instead");
  }
  if (!gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM("Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}

/*
 * Auxiliary methods
 */

void JointTrajectoryActionSingleArm::publisher(const ros::TimerEvent &e)
{
  // Read joint velocities from active EGM connection(s).
  estimated_joint_velocities_.clear();
  bool new_data = false;
  proto::InterfaceData data = egm_interface_.retriveCurrentData(&new_data);
  if(data.has_feedback() && data.feedback().has_joints())
  {
    proto::JointSpace current_joints = data.feedback().joints();

    for(size_t i = 0; i < current_joints.speed_size(); ++i)
    {
      estimated_joint_velocities_.push_back(current_joints.speed().Get(i));
    }

    for(size_t i = 0; i < current_joints.external_speed_size(); ++i)
    {
      estimated_joint_velocities_.push_back(current_joints.external_speed().Get(i));
    }
  }

  // Try to retrive joint data via RWS.
  std::vector<double> all_joint_pos;
  if(p_rws_interface_ != NULL && rws_connection_ready_)
  {
    rws_interface::JointData current_joints;
    if(p_rws_interface_->getData(task_name_, &current_joints))
    {
      all_joint_pos = current_joints.getJointValues();
    }
  }

  if(all_joint_pos.size() == joint_names_.size())
  {
    for(size_t i = 0; i < all_joint_pos.size(); ++i)
    {
      all_joint_pos.at(i) *= conversions::DEG_TO_RAD;
    }

    pub_handler_.publishInformation(all_joint_pos);
  }
}

void JointTrajectoryActionSingleArm::rwsWatchdogRobotSpecific()
{
  EGMData egm_data;
  if(p_rws_interface_->getData(task_name_, &egm_data))
  {
    if(std::abs(egm_data.getMaxSpeedDeviation() - max_moveit_joint_velocity_*conversions::RAD_TO_DEG) < 0.1)
    {
      rws_connection_ready_ = true;
    }
    else
    {
      egm_data.setCondTime(DEFAULT_EGM_CONDITION_TIME_);
      egm_data.setMaxSpeedDeviation(max_moveit_joint_velocity_*conversions::RAD_TO_DEG);
      p_rws_interface_->setData(task_name_, egm_data);
    }
  }
}

} // joint_trajectory_action
} // abb
