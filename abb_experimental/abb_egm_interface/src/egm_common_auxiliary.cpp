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

#include <boost/math/quaternion.hpp>

#include "abb_egm_interface/egm_common_auxiliary.h"
#include "abb_egm_interface/egm_common_math.h"

namespace abb
{
namespace egm_interface
{

using namespace egm_common_values::robot_controller;
using namespace egm_common_values::conversions;

/****************************************************************************************
 * Auxiliary functions
 */

void truncate(proto::JointSpace* joints, const int size)
{
  if (joints->position_size() > size)
  {
    joints->mutable_position()->Truncate(size);
  }

  if (joints->speed_size() > size)
  {
    joints->mutable_speed()->Truncate(size);
  }

  if (joints->acceleration_size() > size)
  {
    joints->mutable_acceleration()->Truncate(size);
  }
}

void fillPosition(proto::JointSpace* joints, const int size, const proto::JointSpace feedback)
{
  int initial_size = joints->position_size();
  for (int i = 0; i < size - initial_size; ++i)
  {
    joints->add_position(feedback.position().Get(i + initial_size));
  }
}

void fillSpeedAndAcceleration(proto::JointSpace* joints, const int size)
{
  int initial_size = joints->speed_size();
  for (int i = 0; i < size - initial_size; ++i)
  {
    joints->add_speed(0.0);
  }

  initial_size = joints->acceleration_size();
  for (int i = 0; i < size - initial_size; ++i)
  {
    joints->add_acceleration(0.0);
  }
}

void truncateExternal(proto::JointSpace* joints, const int size)
{
  if (joints->external_position_size() > size)
  {
    joints->mutable_external_position()->Truncate(size);
  }

  if (joints->external_speed_size() > size)
  {
    joints->mutable_external_speed()->Truncate(size);
  }

  if (joints->external_acceleration_size() > size)
  {
    joints->mutable_external_acceleration()->Truncate(size);
  }
}

void fillExternalPosition(proto::JointSpace* joints, const int size, const proto::JointSpace feedback)
{
  int initial_size = joints->external_position_size();
  for (int i = 0; i < size - initial_size; ++i)
  {
    joints->add_external_position(feedback.external_position().Get(i + initial_size));
  }
}

void fillExternalSpeedAndAcceleration(proto::JointSpace* joints, const int size)
{
  int initial_size = joints->external_speed_size();
  for (int i = 0; i < size - initial_size; ++i)
  {
    joints->add_external_speed(0.0);
  }

  initial_size = joints->external_acceleration_size();
  for (int i = 0; i < size - initial_size; ++i)
  {
    joints->add_external_acceleration(0.0);
  }
}

void fillPosition(proto::Cartesian* cartesian, const proto::Cartesian feedback)
{
  if (!cartesian->has_x())
  {
    cartesian->set_x(feedback.x());
  }

  if (!cartesian->has_y())
  {
    cartesian->set_y(feedback.y());
  }

  if (!cartesian->has_z())
  {
    cartesian->set_z(feedback.z());
  }
}

void fillQuaternion(proto::Quaternion* quaternion, const proto::Quaternion feedback)
{
  if (!quaternion->has_u0())
  {
    quaternion->set_u0(feedback.u0());
  }

  if (!quaternion->has_u1())
  {
    quaternion->set_u1(feedback.u1());
  }

  if (!quaternion->has_u2())
  {
    quaternion->set_u2(feedback.u2());
  }

  if (!quaternion->has_u3())
  {
    quaternion->set_u3(feedback.u3());
  }

  normalizeQuaternion(quaternion);
}

void fillSpeedAndAcceleration(proto::CartesianSpace* cartesian)
{
  int initial_size = cartesian->speed().value_size();
  for (int i = 0; i < DEFAULT_NUMBER_OF_ROBOT_JOINTS - initial_size; ++i)
  {
    cartesian->mutable_speed()->add_value(0.0);
  }

  if (!cartesian->acceleration().has_x())
  {
    cartesian->mutable_acceleration()->set_x(0.0);
  }

  if (!cartesian->acceleration().has_y())
  {
    cartesian->mutable_acceleration()->set_y(0.0);
  }

  if (!cartesian->acceleration().has_z())
  {
    cartesian->mutable_acceleration()->set_z(0.0);
  }
}

void validateInputData(proto::JointSpace* input_data, const proto::JointSpace feedback)
{
  int robot_joints_size = feedback.position_size();
  int external_joints_size = feedback.external_position_size();

  // Make sure that there are the right number of joint elements.
  truncate(input_data, robot_joints_size);
  truncateExternal(input_data, external_joints_size);

  fillPosition(input_data, robot_joints_size, feedback);
  fillExternalPosition(input_data, external_joints_size, feedback);

  // Make sure that there are the right number of joint speed and acceleration elements.
  fillSpeedAndAcceleration(input_data, robot_joints_size);
  fillExternalSpeedAndAcceleration(input_data, external_joints_size);
}

void validateInputData(proto::CartesianSpace* input_data, const proto::CartesianSpace feedback)
{
  if (input_data->has_position())
  {
    fillPosition(input_data->mutable_position(), feedback.position());
  }
  else
  {
    input_data->mutable_position()->CopyFrom(feedback.position());
  }

  if (input_data->has_euler_orientation())
  {
    if (input_data->euler_orientation().has_x() &&
        input_data->euler_orientation().has_y() &&
        input_data->euler_orientation().has_z())
    {
      eulerZYXToQuaternion(input_data->euler_orientation(), input_data->mutable_quaternion_orientation());
      input_data->clear_euler_orientation();
    }
  }

  if (input_data->has_quaternion_orientation())
  {
    fillQuaternion(input_data->mutable_quaternion_orientation(), feedback.quaternion_orientation());
  }
  else
  {
    input_data->mutable_quaternion_orientation()->CopyFrom(feedback.quaternion_orientation());
  }

  // Make sure that there are the right number of speed and acceleration elements.
  fillSpeedAndAcceleration(input_data);
}

void estimateSpeed(proto::JointSpace* estimate,
                   const proto::JointSpace current,
                   const proto::JointSpace previous,
                   const double estimated_sample_time)
{
  double delta_speed = 0.0;

  // Robot joints.
  for (int i = 0; i < current.position_size() && i < previous.position_size(); ++i)
  {
    delta_speed = (current.position(i) - previous.position(i));
    estimate->add_speed(delta_speed / estimated_sample_time);
  }

  // External joints.
  for (int i = 0; i < current.external_position_size() && i < previous.external_position_size(); ++i)
  {
    delta_speed = (current.external_position(i) - previous.external_position(i));
    estimate->add_external_speed(delta_speed / estimated_sample_time);
  }
}

void estimateSpeed(proto::CartesianSpeed* estimate,
                   const proto::CartesianSpace current,
                   const proto::CartesianSpace previous,
                   const double estimated_sample_time)
{
  // Make sure that there are the right number of elements.
  if (estimate->value_size() <= DEFAULT_NUMBER_OF_ROBOT_JOINTS)
  {
    int size = estimate->value_size();
    for (int i = 0; i < DEFAULT_NUMBER_OF_ROBOT_JOINTS - size; ++i)
    {
      estimate->add_value(0.0);
    }
  }
  else
  {
    estimate->mutable_value()->Truncate(DEFAULT_NUMBER_OF_ROBOT_JOINTS);
  }

  // TCP velocity.
  if (current.has_position() && previous.has_position())
  {
    if (current.position().has_x() && previous.position().has_x())
    {
      estimate->set_value(0, (current.position().x() - previous.position().x()) / estimated_sample_time);
    }

    if (current.position().has_y() && previous.position().has_y())
    {
      estimate->set_value(1, (current.position().y() - previous.position().y()) / estimated_sample_time);
    }

    if (current.position().has_z() && previous.position().has_z())
    {
      estimate->set_value(2, (current.position().z() - previous.position().z()) / estimated_sample_time);
    }
  }

  // Angular velocity.
  if (current.has_quaternion_orientation() && previous.has_quaternion_orientation())
  {
    if ((current.quaternion_orientation().has_u0() && previous.quaternion_orientation().has_u0()) &&
        (current.quaternion_orientation().has_u1() && previous.quaternion_orientation().has_u1()) &&
        (current.quaternion_orientation().has_u2() && previous.quaternion_orientation().has_u2()) &&
        (current.quaternion_orientation().has_u3() && previous.quaternion_orientation().has_u3()))
    {
      /*
       * Estimate the angular velocity.
       * See wikipedia for more information: https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions.
       * Note: Only valid for orientations, for the same object, at two points close in time.
       * Also assumes constant angular velocity between the points. 
       */
      boost::math::quaternion<double> q1(previous.quaternion_orientation().u0(),
                                         previous.quaternion_orientation().u1(),
                                         previous.quaternion_orientation().u2(),
                                         previous.quaternion_orientation().u3());

      boost::math::quaternion<double> q2(current.quaternion_orientation().u0(),
                                         current.quaternion_orientation().u1(),
                                         current.quaternion_orientation().u2(),
                                         current.quaternion_orientation().u3());

      boost::math::quaternion<double> estimated_angular_velocity = (2.0*(q2 - q1) / estimated_sample_time)
                                                                    *boost::math::conj(q1)*RAD_TO_DEG;

      estimate->set_value(3, estimated_angular_velocity.R_component_2());
      estimate->set_value(4, estimated_angular_velocity.R_component_3());
      estimate->set_value(5, estimated_angular_velocity.R_component_4());
    }
  }
}

} // end namespace egm_interface
} // end namespace abb
