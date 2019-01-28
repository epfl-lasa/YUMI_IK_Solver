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

#include "abb_egm_interface/egm_message_manager.h"

namespace abb
{
namespace egm_interface
{

using namespace egm_common_values::robot_controller;
using namespace egm_common_values::conversions;

/****************************************************************************************
 * Struct definitions: EGMMessageManager::MessageContainer
 */

EGMMessageManager::MessageContainer::MessageContainer(MessageTypes type)
:
type_(type)
{
  switch (type)
  {
    case EGM:
    {
      p_egm_robot_.reset(new abb::egm::EgmRobot());
      p_egm_sensor_.reset(new abb::egm::EgmSensor());
    }
    break;
  }
}




/****************************************************************************************
 * Class definitions: EGMMessageManager
 */

/********************************************
 * Primary methods
 */

EGMMessageManager::EGMMessageManager(const EGMInterfaceConfiguration::RobotAxes axes,
                                     const EGMInterfaceConfiguration::CommunicationSettings settings,
                                     const MessageTypes type)
:
axes_(axes),
settings_(settings),
message_(type)
{}

void EGMMessageManager::parseFromString(const std::string data, const int bytes_transferred)
{
  switch (message_.type_)
  {
    case EGM:
    {
      message_.p_egm_robot_->ParseFromArray(data.c_str(), bytes_transferred);
    }
    break;
  }
}

std::string EGMMessageManager::serializeToString()
{
  std::string send_buffer = "";

  switch (message_.type_)
  {
    case EGM:
    {
      message_.p_egm_sensor_->SerializeToString(&send_buffer);
    }
    break;
  }

  return send_buffer;
}

void EGMMessageManager::updateSettings(const EGMInterfaceConfiguration::RobotAxes axes,
                                       const EGMInterfaceConfiguration::CommunicationSettings settings)
{
  axes_ = axes;
  settings_ = settings;
}

void EGMMessageManager::constructReply(const size_t sequence_number, proto::TrajectoryPoint target)
{
  message_.p_egm_sensor_->Clear();
  constructReplyHeader(sequence_number, 0);

  switch (message_.type_)
  {
    case EGM:
    {
      constructReplyJointSpaceBody(target.joints());
      constructReplyCartesianSpaceBody(target.cartesian());
    }
    break;
  }
}

void EGMMessageManager::constructReplyHeader(const size_t sequence_number, const size_t timestamp)
{
  switch (message_.type_)
  {
    case EGM:
    {
      abb::egm::EgmHeader* header = message_.p_egm_sensor_->mutable_header();
      header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
      header->set_seqno((google::protobuf::uint32) sequence_number);
      header->set_tm((google::protobuf::uint32) timestamp);
    }
    break;
  }
}

void EGMMessageManager::constructReplyJointSpaceBody(const proto::JointSpace target)
{
  if (target.position_size() > 0 || target.external_position_size() > 0)
  {
    if (settings_.use_position)
    {
      abb::egm::EgmPlanned* planned = message_.p_egm_sensor_->mutable_planned();
      abb::egm::EgmJoints* position = planned->mutable_joints();
      abb::egm::EgmJoints* external_position = planned->mutable_externaljoints();

      // If using a seven axes robot (e.g. IRB14000): Map to special case.
      if (axes_ == EGMInterfaceConfiguration::Seven)
      {
        if (target.position_size() == DEFAULT_NUMBER_OF_ROBOT_JOINTS &&
            target.external_position_size() >= 1)
        {
          position->add_joints(target.position().Get(0));
          position->add_joints(target.position().Get(1));
          position->add_joints(target.position().Get(3));
          position->add_joints(target.position().Get(4));
          position->add_joints(target.position().Get(5));
          position->add_joints(target.external_position().Get(0));
          external_position->add_joints(target.position().Get(2));
        }

        for (int i = 1; i < target.external_position_size(); ++i)
        {
          external_position->add_joints(target.external_position().Get(i));
        }
      }
      else
      {
        for (int i = 0; i < target.position_size(); ++i)
        {
          position->add_joints(target.position().Get(i));
        }

        for (int i = 0; i < target.external_position_size(); ++i)
        {
          external_position->add_joints(target.external_position().Get(i));
        }
      }
    }

    if (settings_.use_speed)
    {
      abb::egm::EgmSpeedRef* speed_reference = message_.p_egm_sensor_->mutable_speedref();
      abb::egm::EgmJoints* speed = speed_reference->mutable_joints();
      abb::egm::EgmJoints* external_speed = speed_reference->mutable_externaljoints();

      // If using a seven axes robot (e.g. IRB14000): Map to special case.
      if (axes_ == EGMInterfaceConfiguration::Seven)
      {
        if (target.speed_size() == DEFAULT_NUMBER_OF_ROBOT_JOINTS &&
            target.external_speed_size() >= 1)
        {
          speed->add_joints(target.speed().Get(0));
          speed->add_joints(target.speed().Get(1));
          speed->add_joints(target.speed().Get(3));
          speed->add_joints(target.speed().Get(4));
          speed->add_joints(target.speed().Get(5));
          speed->add_joints(target.external_speed().Get(0));
          external_speed->add_joints(target.speed().Get(2));
        }

        for (int i = 1; i < target.external_position_size(); ++i)
        {
          external_speed->add_joints(target.external_position().Get(i));
        }
      }
      else
      {
        for (int i = 0; i < target.speed_size(); ++i)
        {
          speed->add_joints(target.speed().Get(i));
        }

        for (int i = 0; i < target.external_speed_size(); ++i)
        {
          external_speed->add_joints(target.external_speed().Get(i));
        }
      }
    }
  }
}

void EGMMessageManager::constructReplyCartesianSpaceBody(const proto::CartesianSpace target)
{
  if (settings_.use_position)
  {
    abb::egm::EgmPlanned* planned = message_.p_egm_sensor_->mutable_planned();
    abb::egm::EgmPose* pose = planned->mutable_cartesian();
    abb::egm::EgmJoints* external_position = planned->mutable_externaljoints();

    if (target.has_position())
    {
      abb::egm::EgmCartesian* position = pose->mutable_pos();
      position->set_x(target.position().x());
      position->set_y(target.position().y());
      position->set_z(target.position().z());
    }

    if (target.has_euler_orientation())
    {
      abb::egm::EgmEuler* euler = pose->mutable_euler();
      euler->set_x(target.euler_orientation().x());
      euler->set_y(target.euler_orientation().y());
      euler->set_z(target.euler_orientation().z());
    }
    else if (target.has_quaternion_orientation())
    {
      abb::egm::EgmQuaternion* quaternion = pose->mutable_orient();
      quaternion->set_u0(target.quaternion_orientation().u0());
      quaternion->set_u1(target.quaternion_orientation().u1());
      quaternion->set_u2(target.quaternion_orientation().u2());
      quaternion->set_u3(target.quaternion_orientation().u3());
    }
  }

  if (settings_.use_speed)
  {
    abb::egm::EgmSpeedRef* speed_reference = message_.p_egm_sensor_->mutable_speedref();
    abb::egm::EgmCartesianSpeed* cartesian_speed = speed_reference->mutable_cartesians();
    abb::egm::EgmJoints* external_speed = speed_reference->mutable_externaljoints();

    if (target.has_speed())
    {
      for (int i = 0; i < target.speed().value_size(); ++i)
      {
        cartesian_speed->add_value(target.speed().value().Get(i));
      }
    }
  }
}

/********************************************
 * Auxiliary methods
 */

bool EGMMessageManager::isFirstMessage()
{
  bool first_message = false;

  switch (message_.type_)
  {
    case EGM:
    {
      first_message = (message_.p_egm_robot_->header().seqno() == 0 ? true : false);
    }
    break;
  }

  return first_message;
}

size_t EGMMessageManager::getTimestamp()
{
  unsigned int timestamp = 0;

  switch (message_.type_)
  {
    case EGM:
    {
      timestamp = message_.p_egm_robot_->header().tm();
    }
    break;
  }

  return timestamp;
}

proto::Feedback EGMMessageManager::getRobotFeedback()
{
  proto::Feedback feedback;

  feedback.mutable_joints()->CopyFrom(getJointSpaceRobotFeedbackPosition());
  feedback.mutable_cartesian()->CopyFrom(getCartesianSpaceRobotFeedbackPosition());

  return feedback;
}

proto::Planned EGMMessageManager::getRobotPlanned()
{
  proto::Planned planned;

  planned.mutable_joints()->CopyFrom(getJointSpaceRobotPlannedPosition());
  planned.mutable_cartesian()->CopyFrom(getCartesianSpaceRobotPlannedPosition());

  return planned;
}

proto::JointSpace EGMMessageManager::getJointSpaceRobotFeedbackPosition()
{
  proto::JointSpace feedback;

  switch (message_.type_)
  {
    case EGM:
    {
      feedback = parseJoints(message_.p_egm_robot_->feedback().joints(),
                             message_.p_egm_robot_->feedback().externaljoints());
    }
    break;
  }

  return feedback;
}

proto::JointSpace EGMMessageManager::getJointSpaceRobotPlannedPosition()
{
  proto::JointSpace planned;

  switch (message_.type_)
  {
    case EGM:
    {
      planned = parseJoints(message_.p_egm_robot_->planned().joints(),
                            message_.p_egm_robot_->planned().externaljoints());
    }
    break;
  }

  return planned;
}

proto::JointSpace EGMMessageManager::parseJoints(const abb::egm::EgmJoints joints,
                                                 const abb::egm::EgmJoints external_joints)
{
  proto::JointSpace joint_space;

  // If using a seven axes robot (e.g. IRB14000): Map to special case.
  if (axes_ == EGMInterfaceConfiguration::Seven)
  {
    if (joints.joints_size() == DEFAULT_NUMBER_OF_ROBOT_JOINTS &&
        external_joints.joints_size() >= 1)
    {
      joint_space.add_position(joints.joints().Get(0) * RAD_TO_DEG);
      joint_space.add_position(joints.joints().Get(1) * RAD_TO_DEG);
      joint_space.add_position(external_joints.joints().Get(0) * RAD_TO_DEG);
      joint_space.add_position(joints.joints().Get(2) * RAD_TO_DEG);
      joint_space.add_position(joints.joints().Get(3) * RAD_TO_DEG);
      joint_space.add_position(joints.joints().Get(4) * RAD_TO_DEG);
      joint_space.add_external_position(joints.joints().Get(5) * RAD_TO_DEG);
    }

    for (int i = 1; i < external_joints.joints_size(); ++i)
    {
      joint_space.add_external_position(external_joints.joints().Get(i) * RAD_TO_DEG);
    }
  }
  else
  {
    for (int i = 0; i < joints.joints_size(); ++i)
    {
      joint_space.add_position(joints.joints().Get(i) * RAD_TO_DEG);
    }

    for (int i = 0; i < external_joints.joints_size(); ++i)
    {
      joint_space.add_external_position(external_joints.joints().Get(i) * RAD_TO_DEG);
    }
  }

  return joint_space;
}

proto::CartesianSpace EGMMessageManager::getCartesianSpaceRobotFeedbackPosition()
{
  proto::CartesianSpace feedback;

  switch (message_.type_)
  {
    case EGM:
    {
      feedback = parsePose(message_.p_egm_robot_->feedback().cartesian());
    }
    break;
  }

  return feedback;
}

proto::CartesianSpace EGMMessageManager::getCartesianSpaceRobotPlannedPosition()
{
  proto::CartesianSpace planned;

  switch (message_.type_)
  {
    case EGM:
    {
      planned = parsePose(message_.p_egm_robot_->planned().cartesian());
    }
    break;
  }

  return planned;
}

proto::CartesianSpace EGMMessageManager::parsePose(const abb::egm::EgmPose pose)
{
  proto::CartesianSpace cartesian_space;

  if (pose.has_pos())
  {
    proto::Cartesian* position = cartesian_space.mutable_position();
    if (pose.pos().has_x())
    {
      position->set_x(pose.pos().x());
    }
    if (pose.pos().has_y())
    {
      position->set_y(pose.pos().y());
    }
    if (pose.pos().has_z())
    {
      position->set_z(pose.pos().z());
    }
  }

  if (pose.has_orient())
  {
    proto::Quaternion* quaternion = cartesian_space.mutable_quaternion_orientation();
    if (pose.orient().has_u0())
    {
      quaternion->set_u0(pose.orient().u0());
    }
    if (pose.orient().has_u1())
    {
      quaternion->set_u1(pose.orient().u1());
    }
    if (pose.orient().has_u2())
    {
      quaternion->set_u2(pose.orient().u2());
    }
    if (pose.orient().has_u3())
    {
      quaternion->set_u3(pose.orient().u3());
    }
  }

  if (pose.has_euler())
  {
    proto::Euler* euler = cartesian_space.mutable_euler_orientation();
    if (pose.euler().has_x())
    {
      euler->set_x(pose.euler().x());
    }
    if (pose.euler().has_y())
    {
      euler->set_y(pose.euler().y());
    }
    if (pose.euler().has_z())
    {
      euler->set_z(pose.euler().z());
    }
  }

  return cartesian_space;
}

proto::RobotStatus EGMMessageManager::getRobotStatus()
{
  proto::RobotStatus robot_status;

  switch (message_.type_)
  {
    case EGM:
    {
      proto::MotorState* p_motor = robot_status.mutable_motor();
      if (message_.p_egm_robot_->has_motorstate() && message_.p_egm_robot_->motorstate().has_state())
      {
        switch (message_.p_egm_robot_->motorstate().state())
        {
          case egm::EgmMotorState_MotorStateType_MOTORS_UNDEFINED:
          {
            p_motor->set_state(proto::MotorState_MotorStateType_MOTORS_UNDEFINED);
          }
          break;

          case egm::EgmMotorState_MotorStateType_MOTORS_ON:
          {
            p_motor->set_state(proto::MotorState_MotorStateType_MOTORS_ON);
          }
          break;

          case egm::EgmMotorState_MotorStateType_MOTORS_OFF:
          {
            p_motor->set_state(proto::MotorState_MotorStateType_MOTORS_OFF);
          }
          break;
        }
      }
      else
      {
        robot_status.mutable_motor()->set_state(proto::MotorState_MotorStateType_MOTORS_UNDEFINED);
      }

      proto::EgmState* p_egm = robot_status.mutable_egm();
      if (message_.p_egm_robot_->has_mcistate() && message_.p_egm_robot_->mcistate().has_state())
      {
        switch (message_.p_egm_robot_->mcistate().state())
        {
          case egm::EgmMCIState_MCIStateType_MCI_UNDEFINED:
          {
            p_egm->set_state(proto::EgmState_EGMStateType_EGM_UNDEFINED);
          }
          break;

          case egm::EgmMCIState_MCIStateType_MCI_ERROR:
          {
            p_egm->set_state(proto::EgmState_EGMStateType_EGM_ERROR);
          }
          break;

          case egm::EgmMCIState_MCIStateType_MCI_STOPPED:
          {
            p_egm->set_state(proto::EgmState_EGMStateType_EGM_STOPPED);
          }
          break;

          case egm::EgmMCIState_MCIStateType_MCI_RUNNING:
          {
            p_egm->set_state(proto::EgmState_EGMStateType_EGM_RUNNING);
          }
          break;
        }
      }
      else
      {
        robot_status.mutable_egm()->set_state(proto::EgmState_EGMStateType_EGM_UNDEFINED);
      }

      proto::RapidExecState* p_rapid = robot_status.mutable_rapid_execution();
      if (message_.p_egm_robot_->has_rapidexecstate() && message_.p_egm_robot_->rapidexecstate().has_state())
      {
        switch (message_.p_egm_robot_->rapidexecstate().state())
        {
          case egm::EgmRapidCtrlExecState_RapidCtrlExecStateType_RAPID_UNDEFINED:
          {
            p_rapid->set_state(proto::RapidExecState_RapidCtrlStateType_RAPID_UNDEFINED);
          }
          break;

          case egm::EgmRapidCtrlExecState_RapidCtrlExecStateType_RAPID_STOPPED:
          {
            p_rapid->set_state(proto::RapidExecState_RapidCtrlStateType_RAPID_STOPPED);
          }
          break;

          case egm::EgmRapidCtrlExecState_RapidCtrlExecStateType_RAPID_RUNNING:
          {
            p_rapid->set_state(proto::RapidExecState_RapidCtrlStateType_RAPID_RUNNING);
          }
          break;
        }
      }
      else
      {
        p_rapid->set_state(proto::RapidExecState_RapidCtrlStateType_RAPID_UNDEFINED);
      }
    }
    break;
  }

  return robot_status;
}

void EGMMessageManager::logData(std::ofstream& log_stream, const proto::TrajectoryPoint target)
{
  switch (message_.type_)
  {
    case EGM:
    {
      // Timestamp.
      log_stream << message_.p_egm_robot_->header().tm() << ", ";

      // Robot feedback.
      logDataAddJoints(log_stream, message_.p_egm_robot_->feedback().joints());
      logDataAddJoints(log_stream, message_.p_egm_robot_->feedback().externaljoints());
      log_stream << message_.p_egm_robot_->feedback().cartesian().pos().x() * MM_TO_M << ", "
                 << message_.p_egm_robot_->feedback().cartesian().pos().y() * MM_TO_M << ", "
                 << message_.p_egm_robot_->feedback().cartesian().pos().z() * MM_TO_M << ", ";

      // Robot planned.
      logDataAddJoints(log_stream, message_.p_egm_robot_->planned().joints());
      logDataAddJoints(log_stream, message_.p_egm_robot_->planned().externaljoints());
      log_stream << message_.p_egm_robot_->planned().cartesian().pos().x() * MM_TO_M << ", "
                 << message_.p_egm_robot_->planned().cartesian().pos().y() * MM_TO_M << ", "
                 << message_.p_egm_robot_->planned().cartesian().pos().z() * MM_TO_M << ", ";

      // Server references.
      logDataAddJoints(log_stream, target.joints());
      log_stream << target.cartesian().position().x() << ", ";
      log_stream << target.cartesian().position().y() << ", ";
      log_stream << target.cartesian().position().z() << ", ";
      for (int i = 0; i < 3; ++i)
      {
        if (i < target.cartesian().speed().value_size())
        {
          log_stream << target.cartesian().speed().value(i) << ", ";
        }
        else
        {
          log_stream << 0.0 << ", ";
        }
      }
      log_stream << target.cartesian().acceleration().x() << ", ";
      log_stream << target.cartesian().acceleration().y() << ", ";
      log_stream << target.cartesian().acceleration().z() << ", ";
      log_stream << target.cartesian().quaternion_orientation().u0() << ", ";
      log_stream << target.cartesian().quaternion_orientation().u1() << ", ";
      log_stream << target.cartesian().quaternion_orientation().u2() << ", ";
      log_stream << target.cartesian().quaternion_orientation().u3() << ", ";
      for (int i = 3; i < DEFAULT_NUMBER_OF_ROBOT_JOINTS; ++i)
      {
        if (i == DEFAULT_NUMBER_OF_ROBOT_JOINTS - 1)
        {
          if (i < target.cartesian().speed().value_size())
          {
            log_stream << target.cartesian().speed().value(i);
          }
          else
          {
            log_stream << 0.0;
          }
        }
        else
        {
          if (i < target.cartesian().speed().value_size())
          {
            log_stream << target.cartesian().speed().value(i) << ", ";
          }
          else
          {
            log_stream << 0.0 << ", ";
          }
        }
      }

      log_stream << "\n";
      log_stream.flush();
    }
    break;
  }
}

void EGMMessageManager::logDataAddJoints(std::ofstream& log_stream, const egm::EgmJoints joints)
{
  google::protobuf::RepeatedField<double>::const_iterator i;

  // Add the joint data.
  for (i = joints.joints().begin(); i != joints.joints().end(); ++i)
  {
    log_stream << *i * RAD_TO_DEG << ", ";
  }
  logDataAddMockJoints(log_stream, joints.joints_size());
}

void EGMMessageManager::logDataAddJoints(std::ofstream& log_stream, const proto::JointSpace joints)
{
  google::protobuf::RepeatedField<double>::const_iterator i;

  // Server joint position references.
  for (i = joints.position().begin(); i != joints.position().end(); ++i)
  {
    log_stream << *i << ", ";
  }
  logDataAddMockJoints(log_stream, joints.position_size());

  for (i = joints.external_position().begin(); i != joints.external_position().end(); ++i)
  {
    log_stream << *i << ", ";
  }
  logDataAddMockJoints(log_stream, joints.external_position_size());


  // Server joint speed references.
  for (i = joints.speed().begin(); i != joints.speed().end(); ++i)
  {
    log_stream << *i << ", ";
  }
  logDataAddMockJoints(log_stream, joints.speed_size());

  for (i = joints.external_speed().begin(); i != joints.external_speed().end(); ++i)
  {
    log_stream << *i << ", ";
  }
  logDataAddMockJoints(log_stream, joints.external_speed_size());


  // Server joint acceleration references.
  for (i = joints.acceleration().begin(); i != joints.acceleration().end(); ++i)
  {
    log_stream << *i << ", ";
  }
  logDataAddMockJoints(log_stream, joints.acceleration_size());

  for (i =joints.external_acceleration().begin(); i != joints.external_acceleration().end(); ++i)
  {
    log_stream << *i << ", ";
  }
  logDataAddMockJoints(log_stream, joints.external_acceleration_size());
}

void EGMMessageManager::logDataAddMockJoints(std::ofstream& log_stream, const size_t size)
{
  // Add mock values for the missing joint data.
  for (size_t i = size; i < DEFAULT_NUMBER_OF_ROBOT_JOINTS; ++i)
  {
    log_stream << 0.0 << ", ";
  }
}

} // end namespace egm_interface
} // end namespace abb
