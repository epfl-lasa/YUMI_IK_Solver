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

#ifndef ABB_JOINT_TRAJECTORY_ACTIONS_H
#define ABB_JOINT_TRAJECTORY_ACTIONS_H

#include "abb_egm_interface/egm_interface_default.h"
#include "abb_joint_trajectory_action_abstract.h"
#include "abb_rws_interface/rws_interface_yumi.h"

namespace abb
{
namespace joint_trajectory_action
{
/**
 * \brief A joint trajectory action class for a single ABB 6-axis robot.
 */
class JointTrajectoryActionSingleArm : public JointTrajectoryActionAbstract
{
public:
  /**
   * \brief Constructor.
   *
   * \param io_service for managing boost::asio asynchronuos operations.
   * \param robot_ip_address for the robot's IP address.
   * \param rws_port for the robot's Robot Web Services (RWS) HTTP server's port number.
   */
  JointTrajectoryActionSingleArm(boost::asio::io_service& io_service,
                                 std::string robot_ip_address,
                                 std::string rws_port);

private:
  /**
   * \brief Action server goal callback method.
   *
   * \param gh goal handle.
   */
  void goalCB(JointTrajectoryActionServer::GoalHandle & gh);

  /**
   * \brief Publisher callback, used to publish information from the robot controller.
   *
   * \param e time event information.
   */
  void publisher(const ros::TimerEvent &e);

  /**
   * \brief Robot specific method to use in the Robot Web Services (RWS) watchdog callback.
   */
  void rwsWatchdogRobotSpecific();

  /**
   * \brief Internal action server.
   */
  JointTrajectoryActionServer action_server_;

  /**
   * \brief An Externally Gudided Motion (EGM) interface for an ABB robot.
   */
  egm_interface::EGMInterfaceDefault egm_interface_;

  /**
   * \brief Name of the used task in the robot controller.
   */
  std::string task_name_;
}; // class JointTrajectoryActionSingleArm

/**
 * \brief A joint trajectory action class for a dual armed ABB robot (e.g. YuMi).
 */
class JointTrajectoryActionDualArmed : public JointTrajectoryActionAbstract
{
public:
  /**
   * \brief Constructor.
   *
   * \param io_service for managing boost::asio asynchronuos operations.
   * \param robot_ip_address for the robot's IP address.
   * \param rws_port for the robot's Robot Web Services (RWS) HTTP server's port number.
   */
  JointTrajectoryActionDualArmed(boost::asio::io_service& io_service,
                                 std::string robot_ip_address,
                                 std::string rws_port);

private:
  /**
   * \brief An enum for the different planning cases.
   */
  enum PlannedCase
  {
    BothArms, ///< Planning for both arms.
    LeftArm,  ///< Planning for the left arm.
    RightArm  ///< Planning for the right arm.
  };

  /**
   * \brief Default number of joints per arm for dual arm robot (e.g. IRB14000).
   */
  static const size_t IRB14000_NUMBER_OF_JOINTS_ARM = 7;

  /**
   * \brief Action server goal callback method.
   *
   * \param gh goal handle.
   * \param planned_case specifying if the planning was for both arms, left arm or right arm.
   */
  void goalCB(JointTrajectoryActionServer::GoalHandle & gh, const PlannedCase planned_case);

  /**
   * \brief Action server goal callback method.
   *
   * \param gh goal handle.
   */
  void goalCB(JointTrajectoryActionServer::GoalHandle & gh);

  /**
   * \brief Publisher callback, used to publish information from the robot controller.
   *
   * \param e time event information.
   */
  void publisher(const ros::TimerEvent &e);

  /**
   * \brief Robot specific method to use in the Robot Web Services (RWS) watchdog callback.
   */
  void rwsWatchdogRobotSpecific();

  /**
   * \brief Auxillary method for extending the goal to contain values for both arms.
   *
   * \param p_goal trajectory goal.
   * \param current_positions containing the last recieved joints feedback positions.
   * \param planned_case specifying if the planning is for both arms, left arm or right arm.
   */
  void extendGoal(control_msgs::FollowJointTrajectoryGoal* p_goal,
                   std::vector<double> current_positions,
                   const PlannedCase planned_case);

  /**
   * \brief Auxillary method for extending a vector of points to contain values for both arms.
   *
   * \param p_points vector of points to extend.
   * \param positions vector of current positions for the arm to compensate for.
   * \param planned_case specifying if the planning is for both arms, left arm or right arm.
   */
  void extendPoints(std::vector<trajectory_msgs::JointTrajectoryPoint>* p_points,
                     const std::vector<double> positions,
                     const PlannedCase planned_case);

  /**
   * \brief Auxillary method for splitting a goal into two goals.
   * This is used for dual arm robots (e.g. IRB14000) to get one goal
   * for the left arm and one goal for the right arm.
   *
   * \param goal trajectory goal
   * \param p_left_goal trajectory goal for the left arm.
   * \param p_right_goal trajectory goal for the right arm.
   */
  void splitGoal(const control_msgs::FollowJointTrajectoryGoal goal,
                  control_msgs::FollowJointTrajectoryGoal* p_left_goal,
                  control_msgs::FollowJointTrajectoryGoal* p_right_goal);

  /**
   * \brief Internal action server for both arms.
   */
  JointTrajectoryActionServer action_server_arms_;

  /**
   * \brief Internal action server for the left arm.
   */
  JointTrajectoryActionServer action_server_left_arm_;

  /**
   * \brief Internal action server for the right arm.
   */
  JointTrajectoryActionServer action_server_right_arm_;

  /**
   * \brief An Externally Gudided Motion (EGM) interface for an ABB robot.
   */
  egm_interface::EGMInterfaceDefault left_arm_egm_interface_;

  /**
   * \brief An Externally Gudided Motion (EGM) interface for an ABB robot.
   */
  egm_interface::EGMInterfaceDefault right_arm_egm_interface_;

  /**
   * \brief A Robot Web Services (RWS) interface for an ABB YuMi robot.
   */
  boost::shared_ptr<rws_interface::RWSInterfaceYuMi> p_rws_interface_yumi_;

  /**
   * \brief An enum indicating if the planning is for left, right or both arms.
   */
  PlannedCase planned_case_;
}; // class JointTrajectoryActionDualArmed

} // joint_trajectory_action
} // abb

#endif /* ABB_JOINT_TRAJECTORY_ACTIONS_H */
