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
#include "abb_rws_interface/rws_interface_yumi.h"

using namespace abb::egm_interface;
using namespace abb::egm_interface::egm_common_values;
using namespace abb::rws_interface;
using namespace abb::rws_interface::rws_strings;

namespace abb
{
namespace joint_trajectory_action
{
/*
 * Class definitions: JointTrajectoryActionDualArmed
 */

/*
 * Primary methods
 */

JointTrajectoryActionDualArmed::JointTrajectoryActionDualArmed(boost::asio::io_service &io_service,
                                                               std::string robot_ip_address,
                                                               std::string rws_port)
  :
  action_server_arms_(node_,
                      "arms/joint_trajectory_action",
                      boost::bind(&JointTrajectoryActionDualArmed::goalCB, this, _1, BothArms),
                      boost::bind(&JointTrajectoryActionDualArmed::cancelCB, this, _1),
                      false),
  action_server_left_arm_(node_,
                          "left_arm/joint_trajectory_action",
                          boost::bind(&JointTrajectoryActionDualArmed::goalCB, this, _1, LeftArm),
                          boost::bind(&JointTrajectoryActionDualArmed::cancelCB, this, _1),
                          false),
  action_server_right_arm_(node_,
                           "right_arm/joint_trajectory_action",
                           boost::bind(&JointTrajectoryActionDualArmed::goalCB, this, _1, RightArm),
                           boost::bind(&JointTrajectoryActionDualArmed::cancelCB, this, _1),
                           false),
  left_arm_egm_interface_(io_service, egm_common_values::communication::DEFAULT_PORT_NUMBER),
  right_arm_egm_interface_(io_service, egm_common_values::communication::DEFAULT_PORT_NUMBER+1)
{
  p_rws_interface_yumi_.reset(new RWSInterfaceYuMi(robot_ip_address, rws_port));
  p_rws_interface_ = p_rws_interface_yumi_;

  EGMInterfaceConfiguration configuration;
  configuration.basic.use_conditions = false;
  configuration.basic.axes = EGMInterfaceConfiguration::Seven;
  configuration.communication.use_speed = true;
  configuration.simple_interpolation.use_speed = true;
  configuration.simple_interpolation.use_acceleration = true;
  configuration.simple_interpolation.use_interpolation = true;
  left_arm_egm_interface_.setConfiguration(configuration);
  right_arm_egm_interface_.setConfiguration(configuration);

  action_server_arms_.start();
  action_server_left_arm_.start();
  action_server_right_arm_.start();
}

void JointTrajectoryActionDualArmed::goalCB(JointTrajectoryActionServer::GoalHandle & gh,
                                            const PlannedCase planned_case)
{
  planned_case_ = planned_case;
  goalCB(gh);
}

void JointTrajectoryActionDualArmed::goalCB(JointTrajectoryActionServer::GoalHandle & gh)
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

  bool left_arm_ready = p_rws_interface_yumi_->isRobotIdle(system_defined::rapid::task_names::RAPID_TASK_ROB_L);
  ros::Duration(RWS_DELAY_TIME_).sleep();
  bool right_arm_ready = p_rws_interface_yumi_->isRobotIdle(system_defined::rapid::task_names::RAPID_TASK_ROB_R);
  if(!left_arm_ready || !right_arm_ready)
  {
    ROS_ERROR("Joint trajectory action rejected: Robot controller is not ready to recieve a command");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Robot controller is not ready to recieve a command");

    // No point in continuing: already rejected.
    return;
  }

  // Copy and extend the goal handle's goal to contain values for both arms.
  control_msgs::FollowJointTrajectoryGoal goal = *gh.getGoal();
  std::vector<double> current_positions;
  for(size_t i = 0; i < last_trajectory_state_->actual.positions.size(); ++i)
  {
    current_positions.push_back(last_trajectory_state_->actual.positions.at(i));
  }
  extendGoal(&goal, current_positions, planned_case_);

  if (!goal.trajectory.points.empty())
  {
    if (industrial_utils::isSimilar(joint_names_, goal.trajectory.joint_names))
    {
      // Cancels the currently active goal.
      if (has_active_goal_)
      {
        ROS_WARN("Received new goal, canceling current goal");
        abortGoal();
      }

      // Sends the trajectory along to the controller.
      if (withinGoalConstraints(last_trajectory_state_, goal.trajectory))
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

        current_traj_ = goal.trajectory;

        control_msgs::FollowJointTrajectoryGoal left_goal;
        control_msgs::FollowJointTrajectoryGoal right_goal;
        splitGoal(goal, &left_goal, &right_goal);

        if(planned_case_ == LeftArm || planned_case_ == BothArms)
        {
          left_arm_egm_interface_.addTrajectory(left_goal, true);
        }
        if(planned_case_ == RightArm || planned_case_ == BothArms)
        {
          right_arm_egm_interface_.addTrajectory(right_goal, true);
        }

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

void JointTrajectoryActionDualArmed::publisher(const ros::TimerEvent &e)
{
  // Read joint velocities from active EGM connection(s).
  estimated_joint_velocities_.clear();
  bool new_data_left = false;
  bool new_data_right = false;
  proto::InterfaceData left_data = left_arm_egm_interface_.retriveCurrentData(&new_data_left);
  proto::InterfaceData right_data = right_arm_egm_interface_.retriveCurrentData(&new_data_right);
  if(left_data.has_feedback() && left_data.feedback().has_joints())
  {
    proto::JointSpace current_joints = left_data.feedback().joints();

    for(size_t i = 0; i < current_joints.speed_size(); ++i)
    {
      estimated_joint_velocities_.push_back(current_joints.speed().Get(i));
    }

    for(size_t i = 0; i < current_joints.external_speed_size(); ++i)
    {
      estimated_joint_velocities_.push_back(current_joints.external_speed().Get(i));
    }
  }
  if(right_data.has_feedback() && right_data.feedback().has_joints())
  {
    proto::JointSpace current_joints = right_data.feedback().joints();

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
  if(p_rws_interface_yumi_ != NULL && rws_connection_ready_)
  {
    rws_interface::DualJointData current_joints;
    if(p_rws_interface_yumi_->getData(&current_joints))
    {
      std::vector<double> temp = current_joints.left.getJointValues();
      for(size_t i = 0; i < temp.size(); ++i)
      {
        all_joint_pos.push_back(temp.at(i));
      }

      temp = current_joints.right.getJointValues();
      for(size_t i = 0; i < temp.size(); ++i)
      {
        all_joint_pos.push_back(temp.at(i));
      }
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

void JointTrajectoryActionDualArmed::rwsWatchdogRobotSpecific()
{
  DualEGMData egm_data;
  if(p_rws_interface_yumi_->getData(&egm_data))
  {
    if(std::abs(egm_data.left.getMaxSpeedDeviation() - max_moveit_joint_velocity_*conversions::RAD_TO_DEG) < 0.1 &&
       std::abs(egm_data.right.getMaxSpeedDeviation() - max_moveit_joint_velocity_*conversions::RAD_TO_DEG) < 0.1)
    {
      rws_connection_ready_ = true;
    }
    else
    {
      egm_data.left.setCondTime(DEFAULT_EGM_CONDITION_TIME_);
      egm_data.left.setMaxSpeedDeviation(max_moveit_joint_velocity_*conversions::RAD_TO_DEG);
      egm_data.right.setCondTime(DEFAULT_EGM_CONDITION_TIME_);
      egm_data.right.setMaxSpeedDeviation(max_moveit_joint_velocity_*conversions::RAD_TO_DEG);
      p_rws_interface_yumi_->setData(egm_data);
    }
  }
}

void JointTrajectoryActionDualArmed::extendGoal(control_msgs::FollowJointTrajectoryGoal* p_goal,
                                                 std::vector<double> current_positions,
                                                 const PlannedCase planned_case)
{
  std::vector<double> missing_joint_positions;

  if(planned_case == LeftArm)
  {
    missing_joint_positions.insert(missing_joint_positions.begin(),
                                   current_positions.begin() + IRB14000_NUMBER_OF_JOINTS_ARM,
                                   current_positions.end());
  }
  else if(planned_case == RightArm)
  {
    missing_joint_positions.insert(missing_joint_positions.begin(),
                                   current_positions.begin(),
                                   current_positions.begin() + IRB14000_NUMBER_OF_JOINTS_ARM);
  }

  switch(planned_case)
  {
    case BothArms:
      // Do nothing. No need to extend the goal since both arms has been planned for.
      break;

    case LeftArm:
    case RightArm:
      // Note: Use the same procedure for both LeftArm and RightArm cases.

      // Set joint names.
      p_goal->trajectory.joint_names = joint_names_;

      // Extend each point in the points vector.
      extendPoints(&(p_goal->trajectory.points), missing_joint_positions, planned_case);

      if(p_goal->path_tolerance.size() > 0 || p_goal->goal_tolerance.size() > 0)
      {
        ROS_WARN("Parsing of tolerances not implemented!");
      }
      break;
  }
}

void JointTrajectoryActionDualArmed::extendPoints(std::vector<trajectory_msgs::JointTrajectoryPoint>* p_points,
                                                   const std::vector<double> positions,
                                                   const PlannedCase planned_case)
{
  std::vector<double> zeros(IRB14000_NUMBER_OF_JOINTS_ARM, 0);

  for(size_t i = 0; i < p_points->size(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint* p_point = &(p_points->at(i));

    // Extend each point in the points vector to also contain the right joints respective left joints.
    // Note: There is no case for BothArms since that should never occur.
    if(planned_case == LeftArm)
    {
      p_point->positions.insert(p_point->positions.end(), positions.begin(), positions.end());
      p_point->velocities.insert(p_point->velocities.end(), zeros.begin(), zeros.end());
      p_point->accelerations.insert(p_point->accelerations.end(), zeros.begin(), zeros.end());

      if(p_point->effort.size() == IRB14000_NUMBER_OF_JOINTS_ARM)
      {
        p_point->effort.insert(p_point->effort.end(), zeros.begin(), zeros.end());
      }
    }
    else if(planned_case == RightArm)
    {
      p_point->positions.insert(p_point->positions.begin(), positions.begin(), positions.end());
      p_point->velocities.insert(p_point->velocities.begin(), zeros.begin(), zeros.end());
      p_point->accelerations.insert(p_point->accelerations.begin(), zeros.begin(), zeros.end());

      if(p_point->effort.size() == IRB14000_NUMBER_OF_JOINTS_ARM)
      {
        p_point->effort.insert(p_point->effort.begin(), zeros.begin(), zeros.end());
      }
    }
  }
}

void JointTrajectoryActionDualArmed::splitGoal(const control_msgs::FollowJointTrajectoryGoal goal,
                                                control_msgs::FollowJointTrajectoryGoal* left_goal,
                                                control_msgs::FollowJointTrajectoryGoal* right_goal)
{
  // Only care about trajectory points (only thing used in the EGM interfaces).
  // Also assume that the left arm's joints are first in the trajectory goal.
  for(size_t i = 0; i < goal.trajectory.points.size(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint point = goal.trajectory.points.at(i);
    trajectory_msgs::JointTrajectoryPoint left_point;
    trajectory_msgs::JointTrajectoryPoint right_point;

    left_point.time_from_start = point.time_from_start;
    right_point.time_from_start = point.time_from_start;

    for(size_t j = 0; j < point.positions.size() && j < point.velocities.size() && j < point.accelerations.size(); ++j)
    {
      if(j < IRB14000_NUMBER_OF_JOINTS_ARM)
      {
        left_point.positions.push_back(point.positions.at(j));
        left_point.velocities.push_back(point.velocities.at(j));
        left_point.accelerations.push_back(point.accelerations.at(j));
      }
      else
      {
        right_point.positions.push_back(point.positions.at(j));
        right_point.velocities.push_back(point.velocities.at(j));
        right_point.accelerations.push_back(point.accelerations.at(j));
      }
    }

    left_goal->trajectory.points.push_back(left_point);
    right_goal->trajectory.points.push_back(right_point);
  }
}

} // joint_trajectory_action
} // abb
