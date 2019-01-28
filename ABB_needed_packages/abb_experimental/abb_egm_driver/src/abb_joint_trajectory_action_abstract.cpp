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

#include <math.h>

#include <industrial_robot_client/utils.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>

#include "abb_egm_driver/abb_joint_trajectory_action_abstract.h"

namespace abb
{
namespace joint_trajectory_action
{
/****************************************************************************************
 * Class definitions: JointTrajectoryActionAbstract
 */

const double JointTrajectoryActionAbstract::DEFAULT_GOAL_THRESHOLD_ = 0.01;
const double JointTrajectoryActionAbstract::DEFAULT_ZERO_SPEED_THRESHOLD_ = 0.0001;
const double JointTrajectoryActionAbstract::PUBLISHER_PERIOD_ = 0.1;
const double JointTrajectoryActionAbstract::WATCHD0G_PERIOD_ = 2.0;
const double JointTrajectoryActionAbstract::RWS_WATCHDOG_PERIOD_ = 1.0;
const double JointTrajectoryActionAbstract::RWS_DELAY_TIME_ = 0.01;
const double JointTrajectoryActionAbstract::DEFAULT_EGM_CONDITION_TIME_ = 10.0;

/********************************************
 * Primary methods
 */

JointTrajectoryActionAbstract::JointTrajectoryActionAbstract()
  :
  goal_threshold_(DEFAULT_GOAL_THRESHOLD_),
  has_active_goal_(false),
  trajectory_state_recvd_(false),
  max_moveit_joint_velocity_(0.0),
  rws_connection_ready_(false),
  watchdog_counter_(0)
{
  node_.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);

  // Retrive the joint names.
  if (!industrial_utils::param::getJointNames("controller_joint_names", "robot_description", joint_names_))
  {
    ROS_ERROR("Failed to initialize joint_names.");
  }

  // The controller joint names parameter includes empty joint names for those joints not supported
  // by the controller. These are removed since the trajectory action should ignore these.
  std::remove(joint_names_.begin(), joint_names_.end(), std::string());
  ROS_INFO_STREAM("Filtered joint names to " << joint_names_.size() << " joints");

  max_moveit_joint_velocity_ = findMaxJointVelocity();

  // Setup publisher handler for EGM information and subscribers.
  pub_handler_.init(joint_names_);
  sub_joint_control_state_ = node_.subscribe("feedback_states",
                                             1,
                                             &JointTrajectoryActionAbstract::controllerStateCB,
                                             this);
  sub_robot_status_ = node_.subscribe("robot_status", 1, &JointTrajectoryActionAbstract::robotStatusCB, this);

  // Create timers.
  watchdog_timer_ = node_.createTimer(ros::Duration(WATCHD0G_PERIOD_),
                                      &JointTrajectoryActionAbstract::watchdog,
                                      this);
  publisher_timer_ = node_.createTimer(ros::Duration(PUBLISHER_PERIOD_),
                                       &JointTrajectoryActionAbstract::publisher,
                                       this);
  rws_watchdog_timer_ = node_.createTimer(ros::Duration(RWS_WATCHDOG_PERIOD_),
                                          &JointTrajectoryActionAbstract::rws_watchdog,
                                          this);
}

void JointTrajectoryActionAbstract::cancelCB(JointTrajectoryActionServer::GoalHandle & gh)
{
  ROS_DEBUG("Received action cancel request");
  if (active_goal_ == gh)
  {
    stopEGM();

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
  else
  {
    ROS_WARN("Active goal handle and cancel goal handle do not match, ignoring cancel request");
  }
}

void JointTrajectoryActionAbstract::controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
  ROS_DEBUG("Checking controller state feedback");
  last_trajectory_state_ = msg;
  trajectory_state_recvd_ = true;
  watchdog_counter_ = 0;

  if (!has_active_goal_)
  {
    ROS_DEBUG("No active goal, ignoring feedback");
    return;
  }
  if (current_traj_.points.empty())
  {
    ROS_DEBUG("Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!industrial_utils::isSimilar(joint_names_, msg->joint_names))
  {
    ROS_ERROR("Joint names from the controller don't match our joint names.");
    return;
  }

  // Checking for goal constraints.
  // Checks that we have ended inside the goal constraints and has motion stopped.
  ROS_DEBUG("Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_, current_traj_))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
      {
        ROS_INFO("Inside goal constraints, stopped moving, return success for action");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
        stopEGM();
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status in motion unknown, the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
        stopEGM();
      }
      else
      {
        ROS_DEBUG("Within goal constraints but robot is still moving");
      }
    }
    else
    {
      ROS_INFO("Inside goal constraints, return success for action");
      ROS_WARN("Robot status is not being published the robot driver node and controller code should be updated");
      active_goal_.setSucceeded();
      has_active_goal_ = false;
      stopEGM();
    }
  }
}

void JointTrajectoryActionAbstract::robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  last_robot_status_ = msg; // Caching robot status for later use.
}

/********************************************
 * Auxiliary methods
 */

void JointTrajectoryActionAbstract::watchdog(const ros::TimerEvent &e)
{
  // Some debug logging.
  if (!last_trajectory_state_)
  {
    ROS_DEBUG("Waiting for subscription to joint trajectory state");
  }
  if (!trajectory_state_recvd_)
  {
    ROS_DEBUG("Trajectory state not received since last watchdog");
  }

  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_)
  {
    if (!trajectory_state_recvd_)
    {
      // Last_trajectory_state_ is null if the subscriber never makes a connection.
      if (!last_trajectory_state_)
      {
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else
      {
        ROS_WARN_STREAM("Aborting goal because we haven't heard from the controller in " <<
                        WATCHD0G_PERIOD_ << " seconds");
      }

      abortGoal();
    }
  }

  if (watchdog_counter_ >= WATCHDOG_MAX_MISSES_)
  {
    // Reset the trajectory state received flag.
    trajectory_state_recvd_ = false;
  }
  ++watchdog_counter_;
}

void JointTrajectoryActionAbstract::rws_watchdog(const ros::TimerEvent &e)
{
  // Robot controller status.
  bool rc_auto = false;
  bool rc_rapid_running = false;

  if(p_rws_interface_ != NULL)
  {
    rc_auto = p_rws_interface_->isModeAuto();
    ros::Duration(RWS_DELAY_TIME_).sleep();
    rc_rapid_running = p_rws_interface_->isRAPIDRunning();

    if(rc_auto && rc_rapid_running)
    {
      if(!rws_connection_ready_)
      {
        rwsWatchdogRobotSpecific();
      }
    }
    else
    {
      ROS_WARN("Robot controller is unavailable (it needs to be in auto mode and also have RAPID started)");
      rws_connection_ready_ = false;
    }
  }
}

bool JointTrajectoryActionAbstract::withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                                                          const trajectory_msgs::JointTrajectory & traj)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    int last_point = traj.points.size() - 1;

    industrial_utils::isSimilar(last_trajectory_state_->joint_names,last_trajectory_state_->joint_names);

    if (industrial_robot_client::utils::isWithinRange(last_trajectory_state_->joint_names,
                                                      last_trajectory_state_->actual.positions, traj.joint_names,
                                                      traj.points[last_point].positions,
                                                      goal_threshold_))
    {
      if(estimated_joint_velocities_.size() == joint_names_.size())
      {
        bool almost_zero_speed = true;
        for(size_t i = 0; i < estimated_joint_velocities_.size() && almost_zero_speed; ++i)
        {
          almost_zero_speed = !(std::abs(estimated_joint_velocities_.at(i)) > DEFAULT_ZERO_SPEED_THRESHOLD_);
        }
        rtn = almost_zero_speed;
      }
      else
      {
        rtn = true;
      }
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

void JointTrajectoryActionAbstract::abortGoal()
{
  stopEGM();

  // Marks the current goal as aborted.
  active_goal_.setAborted();
  has_active_goal_ = false;
}

void JointTrajectoryActionAbstract::stopEGM()
{
  bool done = false;

  if(p_rws_interface_ != NULL && rws_connection_ready_)
  {
    for(int i = 0; i < RWS_MAX_NUMBER_OF_SIGNAL_RETRIES_ && !done; ++i)
    {
      done = p_rws_interface_->doEGMStop();
      if(!done)
      {
        ROS_ERROR_STREAM("Failed to send EGM stop signal! [Attempt " << i+1 << "/" <<
                         RWS_MAX_NUMBER_OF_SIGNAL_RETRIES_ << "]");
      }
    }
  }
}

void JointTrajectoryActionAbstract::startEGM()
{
  bool done = false;

  if(p_rws_interface_ != NULL && rws_connection_ready_)
  {
    for(int i = 0; i < RWS_MAX_NUMBER_OF_SIGNAL_RETRIES_ && !done; ++i)
    {
      done = p_rws_interface_->doEGMStartJoint();
      if(!done)
      {
        ROS_ERROR_STREAM("Failed to send EGM start signal! [Attempt " << i+1 << "/" <<
                         RWS_MAX_NUMBER_OF_SIGNAL_RETRIES_ << "]");
      }
    }
  }
}

double JointTrajectoryActionAbstract::findMaxJointVelocity()
{
  double max_velocity = M_PI/180.0;
  double temp_velocity = 0.0;

  for(size_t i = 0; i < joint_names_.size(); ++i)
  {
    node_.param("robot_description_planning/joint_limits/" + joint_names_.at(i) + "/max_velocity", temp_velocity, 0.0);
    if(temp_velocity > max_velocity)
    {
      max_velocity = temp_velocity;
    }
  }

  return max_velocity;
}

} // joint_trajectory_action
} // abb
