/***********************************************************************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, ABB
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include "abb_egm_interface/egm_common.h"

namespace abb
{
namespace egm_interface
{

using namespace egm_common_values::robot_controller;
using namespace egm_common_values::communication;
using namespace egm_common_values::interpolation;
using namespace egm_common_values::logging;
using namespace egm_common_values::conditions;
using namespace egm_common_values::conversions;

/****************************************************************************************
 * Struct definitions: EGMServerData
 */

EGMServerData::EGMServerData()
:
port_number(0),
message(""),
bytes_transferred(0)
{}




/****************************************************************************************
 * Struct definitions: EGMInterfaceConfiguration::BasicSettings
 */

EGMInterfaceConfiguration::BasicSettings::BasicSettings()
:
axes(Six),
use_conditions(true),
execution_mode(Trajectory)
{}




/****************************************************************************************
 * Struct definitions: EGMInterfaceConfiguration::CommunicationSettings
 */

EGMInterfaceConfiguration::CommunicationSettings::CommunicationSettings()
:
use_position(true),
use_speed(false)
{}




/****************************************************************************************
 * Struct definitions: EGMInterfaceConfiguration::InterpolationSettings
 */

EGMInterfaceConfiguration::Limits::Limits(double speed_limit, double acceleration_limit)
:
speed(speed_limit),
acceleration(acceleration_limit)
{}




/****************************************************************************************
 * Struct definitions: EGMInterfaceConfiguration::InterpolationSettings
 */

EGMInterfaceConfiguration::SimpleInterpolationSettings::SimpleInterpolationSettings()
:
use_interpolation(false),
spline_method(Quintic),
use_speed(false),
use_acceleration(false),
joint_limits(DEFAULT_JOINT_SPEED_LIMIT, DEFAULT_JOINT_ACCELERATION_LIMIT),
cartesian_limits(DEFAULT_CARTESIAN_SPEED_LIMIT, DEFAULT_CARTESIAN_ACCELERATION_LIMIT),
max_T(DEFAULT_MAX_T),
ramp_down_time(DEFAULT_RAMP_DOWN_TIME)
{}




/****************************************************************************************
 * Struct definitions: EGMInterfaceConfiguration::LoggingSettings
 */

EGMInterfaceConfiguration::LoggingSettings::LoggingSettings()
:
use_logging(false),
max_time(DEFAULT_MAX_LOGGING_TIME_MAX)
{}




/****************************************************************************************
 * Class definitions: EGMTrajectory
 */

EGMTrajectory::EGMTrajectory(const proto::Trajectory trajectory)
{
  for (size_t i = 0; i < (size_t)trajectory.points_size(); ++i)
  {
    addTrajectoryPointBack(trajectory.points().Get((int) i));
  }
}

#ifdef ROS_VERSION
EGMTrajectory::EGMTrajectory(const control_msgs::FollowJointTrajectoryGoal goal)
{
  size_t i;
  size_t j;
  double last_time = 0.0;
  double temp_acceleration = 0.0;

  for (i = 0; i < goal.trajectory.points.size(); ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint point = goal.trajectory.points.at(i);
    proto::TrajectoryPoint temp_point;

    for (j = 0; j < point.positions.size(); ++j)
    {
      if (j < DEFAULT_NUMBER_OF_ROBOT_JOINTS)
      {
        temp_point.mutable_joints()->add_position(point.positions.at(j)*RAD_TO_DEG);
      }
      else
      {
        temp_point.mutable_joints()->add_external_position(point.positions.at(j)*RAD_TO_DEG);
      }
    }

    for(j = 0; j < point.velocities.size(); ++j)
    {
      if (j < DEFAULT_NUMBER_OF_ROBOT_JOINTS)
      {
        temp_point.mutable_joints()->add_speed(point.velocities.at(j)*RAD_TO_DEG);
      }
      else
      {
        temp_point.mutable_joints()->add_external_speed(point.velocities.at(j)*RAD_TO_DEG);
      }
    }

    for (j = 0; j < point.accelerations.size(); ++j)
    {
      if (i == 0 || i == goal.trajectory.points.size() - 1)
      {
        temp_acceleration = 0.0;
      }
      else
      {
        temp_acceleration = point.accelerations.at(j)*RAD_TO_DEG;
      }

      if (j < DEFAULT_NUMBER_OF_ROBOT_JOINTS)
      {
        temp_point.mutable_joints()->add_acceleration(temp_acceleration);
      }
      else
      {
        temp_point.mutable_joints()->add_external_acceleration(temp_acceleration);
      }
    }

    if (i == goal.trajectory.points.size() - 1)
    {
      temp_point.set_fine(true);
    }
    else
    {
      temp_point.set_fine(false);
    }

    temp_point.set_duration(point.time_from_start.toSec() - last_time);
    last_time = point.time_from_start.toSec();

    addTrajectoryPointBack(temp_point);
  }
}
#endif

void EGMTrajectory::addTrajectoryPointFront(proto::TrajectoryPoint point)
{
  if (checkPoint(&point))
  {
    points_.push_front(point);
  }
}

void EGMTrajectory::addTrajectoryPointBack(proto::TrajectoryPoint point)
{
  if (checkPoint(&point))
  {
    points_.push_back(point);
  }
}

bool EGMTrajectory::retriveNextTrajectoryPoint(proto::TrajectoryPoint* p_point)
{
  bool result = false;

  if (!points_.empty())
  {
    p_point->CopyFrom(points_.front());
    points_.pop_front();
    result = true;
  }

  return result;
}

void EGMTrajectory::copyTo(proto::Trajectory* p_trajectory)
{
  std::deque<proto::TrajectoryPoint>::const_iterator i;

  for (i = points_.begin(); i != points_.end(); ++i)
  {
    p_trajectory->add_points()->CopyFrom(*i);
  }
}

bool EGMTrajectory::checkPoint(proto::TrajectoryPoint* p_point)
{
  if (p_point->has_joints())
  {
    if (p_point->joints().position_size() > DEFAULT_NUMBER_OF_ROBOT_JOINTS)
    {
      p_point->mutable_joints()->mutable_position()->Truncate(DEFAULT_NUMBER_OF_ROBOT_JOINTS);
    }

    if (p_point->joints().speed_size() > DEFAULT_NUMBER_OF_ROBOT_JOINTS)
    {
      p_point->mutable_joints()->mutable_speed()->Truncate(DEFAULT_NUMBER_OF_ROBOT_JOINTS);
    }

    if (p_point->joints().acceleration_size() > DEFAULT_NUMBER_OF_ROBOT_JOINTS)
    {
      p_point->mutable_joints()->mutable_acceleration()->Truncate(DEFAULT_NUMBER_OF_ROBOT_JOINTS);
    }

    if (p_point->joints().external_position_size() > DEFAULT_NUMBER_OF_EXTERNAL_JOINTS)
    {
      p_point->mutable_joints()->mutable_external_position()->Truncate(DEFAULT_NUMBER_OF_EXTERNAL_JOINTS);
    }

    if (p_point->joints().external_speed_size() > DEFAULT_NUMBER_OF_EXTERNAL_JOINTS)
    {
      p_point->mutable_joints()->mutable_external_speed()->Truncate(DEFAULT_NUMBER_OF_EXTERNAL_JOINTS);
    }

    if (p_point->joints().external_acceleration_size() > DEFAULT_NUMBER_OF_EXTERNAL_JOINTS)
    {
      p_point->mutable_joints()->mutable_external_acceleration()->Truncate(DEFAULT_NUMBER_OF_EXTERNAL_JOINTS);
    }
  }

  if (p_point->has_cartesian())
  {
    if (p_point->cartesian().has_speed())
    {
      if (p_point->cartesian().speed().value_size() > DEFAULT_NUMBER_OF_ROBOT_JOINTS)
      {
        p_point->mutable_cartesian()->mutable_speed()->mutable_value()->Truncate(DEFAULT_NUMBER_OF_ROBOT_JOINTS);
      }
    }
  }

  if (!p_point->has_fine())
  {
    p_point->set_fine(true);
  }

  if (!p_point->has_angle_condition())
  {
    p_point->fine() ? p_point->set_angle_condition(DEFAULT_FINE_CONDITION)
                      :
                      p_point->set_angle_condition(DEFAULT_CONDITION);
  }

  if (!p_point->has_position_condition())
  {
    p_point->fine() ? p_point->set_position_condition(DEFAULT_FINE_CONDITION) 
                      :
                      p_point->set_position_condition(DEFAULT_CONDITION);
  }

  if (p_point->has_duration())
  {
    if (p_point->duration() <= 0.0)
    {
      p_point->set_duration(LOWEST_SAMPLE_TIME*10.0);
    }
  }

  return p_point->has_joints() || p_point->has_cartesian();
}

} // end namespace egm_interface
} // end namespace abb
