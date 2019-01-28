/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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

#ifndef ABB_JOINT_TRAJECTORY_ACTION_ABSTRACT_H
#define ABB_JOINT_TRAJECTORY_ACTION_ABSTRACT_H

#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <industrial_msgs/RobotStatus.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "abb_publisher_handler.h"
#include "abb_rws_interface/rws_interface.h"

namespace abb
{
namespace joint_trajectory_action
{
/**
 * \brief An abstract joint trajectory action server class interfacing
 * with ABB robot controllers via Externally Guided Motion (EGM) and
 * Robot Web Services (RWS).
 */
class JointTrajectoryActionAbstract
{
public:
  /**
   * \brief Default constructor.
   */
  JointTrajectoryActionAbstract();

  /**
   * \brief The maximum number of EGM connections to a single robot controller.
   */
  static const size_t MAX_NUMBER_OF_EGM_CONNECTIONS_ = 4;

  /**
   * \brief Begin processing messages and publishing topics.
   */
  void run() { ros::spin(); }

  /**
   * \brief Controller state callback (executed when feedback message
   * received).
   *
   * \param msg joint trajectory feedback message.
   */
  void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg);

protected:
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JointTrajectoryActionServer;

  /**
   * \brief The default goal joint threshold see(goal_threshold). Unit
   * are joint specific (i.e. radians or meters).
   */
  static const double DEFAULT_GOAL_THRESHOLD_;

  /**
   * \brief The default threshold for determinating almost zero speed [deg/s].
   */
  static const double DEFAULT_ZERO_SPEED_THRESHOLD_;

  /**
   * \brief The publisher period (seconds).
   */
  static const double PUBLISHER_PERIOD_;

  /**
   * \brief The watchdog period (seconds).
   */
  static const double WATCHD0G_PERIOD_;

  /**
   * \brief The Robot Web Servcies (RWS) watchdog period (seconds).
   */
  static const double RWS_WATCHDOG_PERIOD_;

  /**
   * \brief The max number of retries for The Robot Web Servcies (RWS) signal functions.
   */
  static const size_t RWS_MAX_NUMBER_OF_SIGNAL_RETRIES_ = 5;

  /**
   * \brief Delay time [s] between consecutive The Robot Web Servcies (RWS) function calls.
   */
  static const double RWS_DELAY_TIME_;

  /**
   * \brief Max number of missed feedback messages.
   */
  static const size_t WATCHDOG_MAX_MISSES_ = 2;

  /**
   * \brief Default value for Externally Guided Motion (EGM) condition time.
   */
  static const double DEFAULT_EGM_CONDITION_TIME_;

  /**
   * \brief Pure virtual action server goal callback method.
   *
   * \param gh goal handle.
   */
  virtual void goalCB(JointTrajectoryActionServer::GoalHandle & gh) = 0;

  /**
   * \brief Action server cancel callback method.
   *
   * \param gh goal handle.
   */
  void cancelCB(JointTrajectoryActionServer::GoalHandle & gh);

  /**
   * \brief Controller status callback (executed when robot status
   *  message received).
   *
   * \param msg robot status message.
   */
  void robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg);

  /**
   * \brief  Pure virtual publisher callback, used to publish information from the robot controller.
   *
   * \param e time event information.
   */
  virtual void publisher(const ros::TimerEvent &e) = 0;

  /**
   * \brief Watchdog callback, used to detect robot driver failures.
   *
   * \param e time event information.
   */
  void watchdog(const ros::TimerEvent &e);

  /**
   * \brief Robot Web Services (RWS) watchdog callback, used to detect/check RWS connection.
   *
   * \param e time event information.
   */
  void rws_watchdog(const ros::TimerEvent &e);

  /**
   * \brief  Pure virtual robot specific method to use in the Robot Web Services (RWS) watchdog callback.
   */
  virtual void rwsWatchdogRobotSpecific() = 0;

  /**
   * \brief Controller status callback (executed when robot status
   *  message received).
   *
   * \param msg trajectory feedback message.
   * \param traj trajectory to test against feedback.
   *
   * \return true if all joints are within goal contraints.
   */
  bool withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                             const trajectory_msgs::JointTrajectory & traj);

  /**
   * \brief Aborts the current action goal and sends a stop command to the robot driver.
   */
  void abortGoal();

  /**
   * \brief Sends a signal to the robot controllor to stop EGM.
   */
  void stopEGM();

  /**
   * \brief Sends a signal to the robot controllor to start EGM.
   */
  void startEGM();

  /**
   * \brief Finds the maximum velocity limit [rad/s] (from MoveIt joint limits).
   *
   * \return double with the max joint limit.
   */
  double findMaxJointVelocity();

  /**
   * \brief The goal joint threshold used for determining if a robot
   * is near it final destination. A single value is used for all joints.
   *
   * NOTE: This value is used in conjunction with the robot inMotion
   * status (see industrial_msgs::RobotStatus) if it exists.
   */
  double goal_threshold_;

  /**
   * \brief The joint names associated with the robot the action is
   * interfacing with. The joint names must be the same as expected
   * by the robot driver.
   */
  std::vector<std::string> joint_names_;

  /**
   * \brief Internal ROS node handle.
   */
  ros::NodeHandle node_;

  /**
   * \brief A publisher object for publishing information from the robot controller.
   */
  publisher_handler::PublisherHandler pub_handler_;

  /**
   * \brief Subscribes to trajectory feedback (typically published by the
   * robot driver).
   */
  ros::Subscriber sub_joint_control_state_;

  /**
   * \brief Subscribes to the robot status (typically published by the
   * robot driver).
   */
  ros::Subscriber sub_robot_status_;

  /**
   * \brief Publisher timer used to publish information from the robot controller.
   */
  ros::Timer publisher_timer_;

  /**
   * \brief Watchdog timer used to fail the action request if the robot
   * driver is not responding.
   */
  ros::Timer watchdog_timer_;

  /**
   * \brief Robot Web Services (RWS) watchdog timer used to detect/check the RWS connection.
   */
  ros::Timer rws_watchdog_timer_;

  /**
   * \brief A Robot Web Services (RWS) interface for ABB robots.
   */
  boost::shared_ptr<rws_interface::RWSInterface> p_rws_interface_;

  /**
   * \brief Indicates action has an active goal.
   */
  bool has_active_goal_;

  /**
   * \brief Cache of the current active goal.
   */
  JointTrajectoryActionServer::GoalHandle active_goal_;

  /**
   * \brief Cache of the current active trajectory.
   */
  trajectory_msgs::JointTrajectory current_traj_;

  /**
   * \brief Indicates trajectory state has been received. Used by
   * watchdog to determine if the robot driver is responding.
   */
  bool trajectory_state_recvd_;

  /**
   * \brief Cache of the last subscribed feedback message.
   */
  control_msgs::FollowJointTrajectoryFeedbackConstPtr last_trajectory_state_;

  /**
   * \brief Cache of the last subscribed status message.
   */
  industrial_msgs::RobotStatusConstPtr last_robot_status_;

  /**
   * \brief The highest value [rad/s] of the max joint velocities (from MoveIt).
   */
  double max_moveit_joint_velocity_;

  /**
   * \brief Indicates if the Robot Web Services (RWS) are ready to use.
   */
  bool rws_connection_ready_;

  /**
   * \brief An estimate of the current joint velocities
   * (only available if Externally Guided Motion (EGM) is active).
   */
  std::vector<double> estimated_joint_velocities_;

  /**
   * \brief A counter for the watchdog to count the number of missed feedback messages.
   */
  size_t watchdog_counter_;
}; // class JointTrajectoryActionAbstract

} // joint_trajectory_action
} // abb

#endif /* ABB_JOINT_TRAJECTORY_ACTION_ABSTRACT_H */
