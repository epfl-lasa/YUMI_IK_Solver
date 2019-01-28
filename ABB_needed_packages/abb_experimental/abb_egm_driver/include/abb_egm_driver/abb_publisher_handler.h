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

#ifndef ABB_PUBLISHER_HANDLER_H
#define ABB_PUBLISHER_HANDLER_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>

namespace abb
{
namespace publisher_handler
{

/**
 * \brief Publisher handler for publishing robot controller information (e.g. joint positions and robot status).
 */
class PublisherHandler
{
public:
  /**
   * \brief Class initializer.
   *
   * \param joint_names list of joint-names for msg-publishing.
   *   - Count and order should match data from robot connection.
   *   - Use blank-name to exclude a joint from publishing.
   */
  void init(std::vector<std::string> &joint_names);

  /**
   * \brief Callback executed upon receiving a joint message.
   *
   * \param all_joint_pos joint positions, in count/order matching robot connection.
   *
   * \return true on success, false otherwise.
   */
  bool publishInformation(const std::vector<double>& all_joint_pos);

protected:
  /**
   * \brief Internal ROS node handle.
   *
   */
  ros::NodeHandle node_;

  /**
   * \brief Joint names of the robot.
   */
  std::vector<std::string> all_joint_names_;

  /**
   * \brief Publisher for trajectory feedback.
   */
  ros::Publisher pub_joint_control_state_;

  /**
   * \brief Publisher for joint states.
   */
  ros::Publisher pub_joint_sensor_state_;

  /**
   * \brief Publisher for robot status.
   */
  ros::Publisher pub_robot_status_;

  /**
   * \brief Convert joint message into publish message-types.
   *
   * \param[in] all_joint_pos joint positions, in count/order matching robot connection.
   * \param[out] control_state FollowJointTrajectoryFeedback message for ROS publishing.
   * \param[out] sensor_state JointState message for ROS publishing.
   *
   * \return true on success, false otherwise
   */
  bool createMessages(const std::vector<double>& all_joint_pos,
                      control_msgs::FollowJointTrajectoryFeedback* control_state,
                      sensor_msgs::JointState* sensor_state);

  /**
   * \brief Transform joint positions before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in] pos_in joint positions, exactly as passed from robot connection.
   * \param[out] pos_out transformed joint positions (in same order/count as input positions).
   *
   * \return true on success, false otherwise.
   */
  bool transform(const std::vector<double>& pos_in, std::vector<double>* pos_out)
  {
    *pos_out = pos_in;  // By default, no transform is applied.
    return true;
  }

  /**
   * \brief Select specific joints for publishing.
   *
   * \param[in] all_joint_pos joint positions, in count/order matching robot connection.
   * \param[in] all_joint_names joint names, matching all_joint_pos.
   * \param[out] pub_joint_pos joint positions selected for publishing.
   * \param[out] pub_joint_names joint names selected for publishing.
   *
   * \return true on success, false otherwise.
   */
  bool select(const std::vector<double>& all_joint_pos,
              const std::vector<std::string>& all_joint_names,
              std::vector<double>* pub_joint_pos,
              std::vector<std::string>* pub_joint_names);

}; // class PublisherHandler

} // publisher_handler
} // abb

#endif /* ABB_PUBLISHER_HANDLER_H */
