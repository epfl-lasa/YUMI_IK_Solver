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

#include <boost/lexical_cast.hpp>

#include "abb_egm_interface/egm_common_auxiliary.h"
#include "abb_egm_interface/egm_common_math.h"
#include "abb_egm_interface/egm_interface_default.h"

namespace abb
{
namespace egm_interface
{
/****************************************************************************************
 * Struct definitions: EGMInterfaceDefault::Auxiliary
 */

EGMInterfaceDefault::Auxiliary::Auxiliary()
:
estimated_sample_time(0.0),
first_call(true),
mode(EGMInterfaceConfiguration::EGMJoint),
number_of_logged_messages(0),
sequence_number(0),
new_message(false)
{};

void EGMInterfaceDefault::Auxiliary::initializeData(EGMMessageManager* p_egm_messages)
{
  estimated_sample_time = egm_common_values::robot_controller::LOWEST_SAMPLE_TIME;
  first_call            = false;
  initial_feedback      = p_egm_messages->getRobotFeedback(); // For the demo function.
  mode                  = EGMInterfaceConfiguration::EGMJoint;
  sequence_number       = 0;
}

void EGMInterfaceDefault::Auxiliary::updateInterfaceData(const MotionStep& motion_step,
                                                         const PlannedMotion& planned_motion)
{
  // Lock the interface's auxiliary data mutex. It is released when the lock goes out of scope.
  boost::lock_guard<boost::mutex> lock(mutex);

  interface_data.Clear();
  
  // Add current feedback and planned values.
  interface_data.mutable_feedback()->CopyFrom(motion_step.current.feedback);
  interface_data.mutable_planned()->CopyFrom(motion_step.current.planned);
  
  // Add the current planned trajectory.
  interface_data.mutable_currently_planned_trajectory()->add_points()->CopyFrom(motion_step.goal.target);
  if (planned_motion.hasCurrentTrajectory())
  {
    planned_motion.trajectories.p_current->copyTo(interface_data.mutable_currently_planned_trajectory());
  }

  // Add a speed estimation.
  updateInterfaceDataSpeed(motion_step);

  // Add a the robot status.
  interface_data.mutable_robot_status()->CopyFrom(motion_step.robot_status);
}

void EGMInterfaceDefault::Auxiliary::updateInterfaceDataSpeed(const MotionStep& motion_step)
{
  if (!(estimated_sample_time > 0))
  {
    estimated_sample_time = egm_common_values::robot_controller::LOWEST_SAMPLE_TIME;
  }

  /*------------------------------------------------------
   * Estimate joint speed.
   */

  // Feedback speed.
  if (motion_step.current.feedback.has_joints() && motion_step.previous.feedback.has_joints())
  {
    estimateSpeed(interface_data.mutable_feedback()->mutable_joints(),
                  motion_step.current.feedback.joints(),
                  motion_step.previous.feedback.joints(),
                  estimated_sample_time);
  }

  // Planned speed.
  if (motion_step.current.planned.has_joints() && motion_step.previous.planned.has_joints())
  {
    estimateSpeed(interface_data.mutable_planned()->mutable_joints(),
                  motion_step.current.planned.joints(),
                  motion_step.previous.planned.joints(),
                  estimated_sample_time);
  }

  /*------------------------------------------------------
   * Estimate Cartesian speed.
   */

  // Feedback speed.
  if (motion_step.current.feedback.has_cartesian() && motion_step.previous.feedback.has_cartesian())
  {
    estimateSpeed(interface_data.mutable_feedback()->mutable_cartesian()->mutable_speed(),
                  motion_step.current.feedback.cartesian(),
                  motion_step.previous.feedback.cartesian(),
                  estimated_sample_time);
  }

  // Planned speed.
  if (motion_step.current.planned.has_cartesian() && motion_step.previous.planned.has_cartesian())
  {
    estimateSpeed(interface_data.mutable_planned()->mutable_cartesian()->mutable_speed(),
                  motion_step.current.planned.cartesian(),
                  motion_step.previous.planned.cartesian(),
                  estimated_sample_time);
  }
}




/****************************************************************************************
 * Struct definitions: EGMInterfaceDefault::Configuration
 */

EGMInterfaceDefault::Configuration::Configuration(EGMInterfaceConfiguration initial_configuration)
:
active(initial_configuration),
pending_update(false),
shared(initial_configuration)
{}




/****************************************************************************************
* Struct definitions: EGMInterfaceDefault::DirectReferences
*/

EGMInterfaceDefault::DirectReferences::DirectReferences()
:
read_data_ready(false),
write_data_ready(false)
{}

void EGMInterfaceDefault::DirectReferences::initializeData()
{
  boost::lock_guard<boost::mutex> lock(read_mutex);
  boost::lock_guard<boost::mutex> lock2(write_mutex);

  read_data_ready = false;
  write_data_ready = false;
}

void EGMInterfaceDefault::DirectReferences::wait_for_data()
{
  boost::unique_lock<boost::mutex> lock(read_mutex);
  while (!read_data_ready)
  {
    read_condition_variable.wait(lock);
  }
}

void EGMInterfaceDefault::DirectReferences::read(proto::Feedback* p_feedback, proto::RobotStatus* p_robot_status)
{
  boost::lock_guard<boost::mutex> lock(read_mutex);
  p_feedback->CopyFrom(feedback);
  p_robot_status->CopyFrom(robot_status);
  read_data_ready = false;
}

void EGMInterfaceDefault::DirectReferences::updateTarget(proto::JointSpace references)
{
  boost::lock_guard<boost::mutex> lock(read_mutex);
  boost::lock_guard<boost::mutex> lock2(write_mutex);

  validateInputData(&references, feedback.joints());

  target.Clear();
  target.mutable_joints()->CopyFrom(references);
  if (feedback.has_cartesian())
  {
    target.mutable_cartesian()->CopyFrom(feedback.cartesian());
  }
  write_data_ready = true;
  write_condition_variable.notify_all();
}

void EGMInterfaceDefault::DirectReferences::updateTarget(proto::CartesianSpace references)
{
  boost::lock_guard<boost::mutex> lock(read_mutex);
  boost::lock_guard<boost::mutex> lock2(write_mutex);

  validateInputData(&references, feedback.cartesian());

  target.Clear();
  target.mutable_cartesian()->CopyFrom(references);
  if (feedback.has_joints())
  {
    target.mutable_joints()->CopyFrom(feedback.joints());
  }
  write_data_ready = true;
  write_condition_variable.notify_all();
}

void EGMInterfaceDefault::DirectReferences::updateReferences(proto::TrajectoryPoint* p_references)
{
  if (target.has_joints())
  {
    p_references->mutable_joints()->CopyFrom(target.joints());
  }

  if (target.has_cartesian())
  {
    p_references->mutable_cartesian()->CopyFrom(target.cartesian());
  }
  write_data_ready = false;
}




/****************************************************************************************
 * Struct definitions: EGMInterfaceDefault::MotionStep
 */

void EGMInterfaceDefault::MotionStep::checkQuaternions()
{
  proto::Quaternion q1 = current.references.cartesian().quaternion_orientation();
  proto::Quaternion q2 = goal.target.cartesian().quaternion_orientation();

  // Check the quaternion dot product and negate the targeted quaternion if the dot product is negative.
  // I.e. this is to make the Slerp interpolation take the short way.
  // See https://en.wikipedia.org/wiki/Slerp for more information.
  if (dotProduct(q1, q2) < 0.0)
  {
    goal.target.mutable_cartesian()->mutable_quaternion_orientation()->set_u0(-q2.u0());
    goal.target.mutable_cartesian()->mutable_quaternion_orientation()->set_u1(-q2.u1());
    goal.target.mutable_cartesian()->mutable_quaternion_orientation()->set_u2(-q2.u2());
    goal.target.mutable_cartesian()->mutable_quaternion_orientation()->set_u3(-q2.u3());
  }
}

bool EGMInterfaceDefault::MotionStep::conditionMet(EGMInterfaceConfiguration::InterfaceModes mode)
{
  bool point_reached = true;

  switch (mode)
  {
    case EGMInterfaceConfiguration::EGMJoint:
    {
      // Robot joints.
      for (int i = 0; i < current.feedback.joints().position_size() && point_reached == true; ++i)
      {
        if (fabs(current.feedback.joints().position().Get(i) -
                 goal.target.joints().position().Get(i)) >
            goal.target.angle_condition())
        {
          point_reached = false;
        }
      }

      // External joints.
      for (int i = 0; i < current.feedback.joints().external_position_size() && point_reached == true; ++i)
      {
        if (fabs(current.feedback.joints().external_position().Get(i) -
                  goal.target.joints().external_position().Get(i)) > 
            goal.target.angle_condition())
        {
          point_reached = false;
        }
      }
    }
    break;

    case EGMInterfaceConfiguration::EGMCartesian:
    {
      // TCP position.
      double delta_x = fabs(current.feedback.cartesian().position().x() - goal.target.cartesian().position().x());
      double delta_y = fabs(current.feedback.cartesian().position().y() - goal.target.cartesian().position().y());
      double delta_z = fabs(current.feedback.cartesian().position().z() - goal.target.cartesian().position().z());
      if (delta_x > goal.target.position_condition() ||
          delta_y > goal.target.position_condition() ||
          delta_z > goal.target.position_condition())
      {
        point_reached = false;
      }

      // TCP orientation.
      double dot_product = dotProduct(current.feedback.cartesian().quaternion_orientation(),
                                      goal.target.cartesian().quaternion_orientation());

      if (1.0 - fabs(dot_product) > egm_common_values::conditions::DEFAULT_FINE_CONDITION)
      {
        point_reached = false;
      }

      // External joints.
      for (int i = 0; i < current.feedback.joints().external_position_size() && point_reached == true; ++i)
      {
        if (fabs(current.feedback.joints().external_position().Get(i) -
                  goal.target.joints().external_position().Get(i)) >
            goal.target.angle_condition())
        {
          point_reached = false;
        }
      }
    }
    break;
  }

  return point_reached;
}

void EGMInterfaceDefault::MotionStep::initializeData(EGMMessageManager* p_egm_messages)
{
  current.timestamp  = (unsigned int) p_egm_messages->getTimestamp();
  previous.timestamp = current.timestamp;

  current.feedback  = p_egm_messages->getRobotFeedback();
  current.planned   = p_egm_messages->getRobotPlanned();
  previous.feedback = current.feedback;
  previous.planned  = current.planned;

  resetReferences();
}

void EGMInterfaceDefault::MotionStep::updateData(EGMMessageManager* p_egm_messages)
{
  current.feedback  = p_egm_messages->getRobotFeedback();
  current.planned   = p_egm_messages->getRobotPlanned();
  current.timestamp = (unsigned int) p_egm_messages->getTimestamp();
}

void EGMInterfaceDefault::MotionStep::updatePrevious()
{
  previous.feedback  = current.feedback;
  previous.planned   = current.planned;
  previous.timestamp = current.timestamp;
}

void EGMInterfaceDefault::MotionStep::validateTarget()
{
  int robot_joints_size = current.feedback.joints().position_size();
  int external_joints_size = current.feedback.joints().external_position_size();

  proto::JointSpace* joint_target;
  proto::JointSpace joint_feedback = current.feedback.joints();
  proto::CartesianSpace* cartesian_target;
  proto::CartesianSpace cartesian_feedback = current.feedback.cartesian();

  #pragma region Validate joint mode
  // Make sure that there are the right number of joint elements.
  if (goal.target.has_joints())
  {
    joint_target = goal.target.mutable_joints();

    truncate(joint_target, robot_joints_size);
    truncateExternal(joint_target, external_joints_size);

    fillPosition(joint_target, robot_joints_size, joint_feedback);
    fillExternalPosition(joint_target, external_joints_size, joint_feedback);
  }
  else
  {
    joint_target = goal.target.mutable_joints();
    joint_target->CopyFrom(joint_feedback);
  }

  // Make sure that there are the right number of joint speed and acceleration elements.
  fillSpeedAndAcceleration(joint_target, robot_joints_size);
  fillExternalSpeedAndAcceleration(joint_target, external_joints_size);
  #pragma endregion

  #pragma region Validate Cartesian mode
  // Make sure that there are the right number of Cartesian elements.
  if (goal.target.has_cartesian())
  {
    cartesian_target = goal.target.mutable_cartesian();

    if (cartesian_target->has_position())
    {
      fillPosition(cartesian_target->mutable_position(), cartesian_feedback.position());
    }
    else
    {
      cartesian_target->mutable_position()->CopyFrom(cartesian_feedback.position());
    }

    if (cartesian_target->has_euler_orientation())
    {
      if (cartesian_target->euler_orientation().has_x() &&
          cartesian_target->euler_orientation().has_y() &&
          cartesian_target->euler_orientation().has_z())
      {
        // Convert ZYX Euler angles to quaternion because it is easier to process later (e.g. interpolate).
        eulerZYXToQuaternion(cartesian_target->euler_orientation(),
                             cartesian_target->mutable_quaternion_orientation());
        cartesian_target->clear_euler_orientation();
      }
    }

    if (cartesian_target->has_quaternion_orientation())
    {
      fillQuaternion(cartesian_target->mutable_quaternion_orientation(),
                     cartesian_feedback.quaternion_orientation());
    }
    else
    {
      cartesian_target->mutable_quaternion_orientation()->CopyFrom(cartesian_feedback.quaternion_orientation());
    }
  }
  else
  {
    cartesian_target = goal.target.mutable_cartesian();
    cartesian_target->CopyFrom(current.feedback.cartesian());
  }

  // Make sure that there are the right number of speed and acceleration elements.
  fillSpeedAndAcceleration(cartesian_target);
  #pragma endregion
}

void EGMInterfaceDefault::MotionStep::resetReferences()
{
  goal.target.Clear();
  validateTarget();
  current.references = goal.target;
}




/****************************************************************************************
 * Struct definitions: EGMInterfaceDefault::PendingMotionEvents
 */

EGMInterfaceDefault::PendingMotionEvents::PendingMotionEvents()
:
do_discard(false),
do_evasion(false),
do_override(false),
do_stop(false)
{};




/****************************************************************************************
 * Struct definitions: EGMInterfaceDefault::PlannedMotion
 */

bool EGMInterfaceDefault::PlannedMotion::retriveNextTrajectoryPoint(proto::TrajectoryPoint* p_point)
{
  bool result = false;

  if (hasCurrentTrajectory())
  {
    result = trajectories.p_current->retriveNextTrajectoryPoint(p_point);
  }

  return result;
}




/****************************************************************************************
 * Class definitions: EGMInterfaceDefault
 */

/********************************************
 * Primary methods
 */

EGMInterfaceDefault::EGMInterfaceDefault(boost::asio::io_service& io_service,
                                         const size_t port_number,
                                         const EGMInterfaceConfiguration initial_configuration)
:
egm_server_(io_service, port_number, this),
configuration_(initial_configuration),
egm_messages_(initial_configuration.basic.axes, initial_configuration.communication),
simple_interpolator_(initial_configuration.simple_interpolation),
current_state_(Normal)
{
  std::string filename = "port_" + boost::lexical_cast<std::string>(port_number) + "_log.txt";
  log_stream_.open(filename.c_str(), std::ios::trunc);
}

EGMInterfaceDefault::~EGMInterfaceDefault()
{
  log_stream_.close();
}

std::string EGMInterfaceDefault::callbackFunction(const EGMServerData server_data)
{
  // 1. Parse the recieved message and setup internal data.
  egm_messages_.parseFromString(server_data.message, (int) server_data.bytes_transferred);
  setupData();

  // 2. Decide the next references.
  switch (configuration_.active.basic.execution_mode)
  {
    case EGMInterfaceConfiguration::Demo:
    {
      demoCreateEGMReferences();
    }
    break;

    case EGMInterfaceConfiguration::Direct:
    {
      {
        boost::lock_guard<boost::mutex> lock(direct_references_.read_mutex);
        direct_references_.feedback.CopyFrom(motion_step_.current.feedback);
        direct_references_.robot_status.CopyFrom(motion_step_.robot_status);
        direct_references_.read_data_ready = true;
        direct_references_.read_condition_variable.notify_all();
      }

      if (motion_step_.robot_status.egm().state() == proto::EgmState_EGMStateType_EGM_RUNNING)
      {
        boost::unique_lock<boost::mutex> lock(direct_references_.write_mutex);
        bool timed_out = false;
        while (!direct_references_.write_data_ready && !timed_out)
        {
          timed_out = !direct_references_. write_condition_variable.timed_wait(lock,
                                                                               boost::posix_time::milliseconds(25));
        }

        if (timed_out)
        {
          motion_step_.resetReferences();
        }
        else
        {
          direct_references_.updateReferences(&motion_step_.current.references);
        }
      }
    }
    break;

    case EGMInterfaceConfiguration::Trajectory:
    {
      updateCurrentTarget();
      processCurrentTarget();
      auxiliary_.updateInterfaceData(motion_step_, planned_motion_);
    }
    break;
  }
  
  // 3. Construct the EGM sensor message.
  egm_messages_.constructReply(auxiliary_.sequence_number++, motion_step_.current.references);

  // 4. Log data to a CSV file (if allowed).
  if (configuration_.active.logging.use_logging &&
      auxiliary_.loggedTime() <= configuration_.active.logging.max_time)
  {
    egm_messages_.logData(log_stream_, motion_step_.current.references);
    ++auxiliary_.number_of_logged_messages;
  }

  // 5. Store information for the next callback and return the serialized reply message.
  motion_step_.updatePrevious();

  return egm_messages_.serializeToString();
}

void EGMInterfaceDefault::setupData()
{
  // Initialize data if it is the first call or if a new EGM communication session has begun.
  if (auxiliary_.first_call || egm_messages_.isFirstMessage())
  {
    // Update the configurations, if it has been requested.
    {
      boost::lock_guard<boost::mutex> lock(configuration_.mutex);
      if (configuration_.pending_update)
      {
        configuration_.active = configuration_.shared;
        configuration_.pending_update = false;

        egm_messages_.updateSettings(configuration_.active.basic.axes, configuration_.active.communication);
        simple_interpolator_.updateSettings(configuration_.active.simple_interpolation);
      }
    }

    auxiliary_.initializeData(&egm_messages_);
    motion_step_.initializeData(&egm_messages_);
    direct_references_.initializeData();

    if (configuration_.active.simple_interpolation.use_interpolation)
    {
      motion_step_.checkQuaternions();
      simple_interpolator_.update(motion_step_.current.references,
                                  motion_step_.goal.target,
                                  auxiliary_.mode);
    }
  }
  else
  {
    motion_step_.updateData(&egm_messages_);
    auxiliary_.estimated_sample_time = motion_step_.estimateSampleTime();
  }

  {
    boost::lock_guard<boost::mutex> lock(auxiliary_.mutex);
    auxiliary_.new_message = true;
  }
  motion_step_.robot_status = egm_messages_.getRobotStatus();
}

void EGMInterfaceDefault::updateCurrentTarget()
{
  // Lock the interface's planned motion mutex. It is released when the lock goes out of scope.
  boost::lock_guard<boost::mutex> lock(planned_motion_.trajectories.mutex);

  // Update the current state (only if the current motion is NOT in the process of being ramped down).
  if (current_state_ != Stopping && current_state_ != Stopped)
  {
    if (pending_events_.do_stop)
    {
      current_state_ = Stop;
    }
    else if (pending_events_.do_override)
    {
      current_state_ = OverrideTrajectory;
    }
    else
    {
      current_state_ = Normal;
    }
  }

  // Handle the current state.
  switch (current_state_)
  {
    case Normal:
    {
      if (planned_motion_.hasCurrentTrajectory())
      {
        if (configuration_.active.basic.use_conditions)
        {
          if (motion_step_.conditionMet(auxiliary_.mode))
          {
            retriveNextTrajectoryPoint();
          }
        }
        else if (configuration_.active.simple_interpolation.use_interpolation)
        {
          if (std::abs(simple_interpolator_.getT() - motion_step_.goal.time_passed) < 
              egm_common_values::robot_controller::LOWEST_SAMPLE_TIME*0.5)
          {
            retriveNextTrajectoryPoint();
          }
        }
        else
        {
          retriveNextTrajectoryPoint();
        }
      }
      else
      {
        if (planned_motion_.primaryQueueEmpty())
        {
          // Do nothing (i.e. use the previous target).
        }
        else
        {
          planned_motion_.popPrimaryQueue();
          retriveNextTrajectoryPoint();
        }
      }
    }
    break;

    case OverrideTrajectory:
    {
      planned_motion_.trajectories.p_current.reset();
      planned_motion_.emptyPrimaryQueue();
      planned_motion_.transferTemporaryToPrimaryQueue();

      if (planned_motion_.primaryQueueEmpty())
      {
        // Should not happen.
      }
      else
      {
        planned_motion_.popPrimaryQueue();
        retriveNextTrajectoryPoint();
      }

      pending_events_.do_override = false;
    }
    break;

    case Stop:
    {
      if (pending_events_.do_discard)
      {
        planned_motion_.trajectories.p_current.reset();
        planned_motion_.emptyPrimaryQueue();
        planned_motion_.transferTemporaryToPrimaryQueue();

        pending_events_.do_discard = false;
      }
      else
      {
        planned_motion_.trajectories.p_current->addTrajectoryPointFront(motion_step_.goal.target);
      }

      motion_step_.checkQuaternions();
      simple_interpolator_.update(motion_step_.current.references,
                                  auxiliary_.mode);
      motion_step_.goal.time_passed = 0.0;

      current_state_ = Stopping;
      pending_events_.do_stop = false;
    }
    break;

    case Stopping:
    {
      // Do nothing.
    }
    break;

    case Stopped:
    {
      if (pending_events_.do_evasion)
      {
        planned_motion_.trajectories.p_current = planned_motion_.trajectories.p_evasive;
        retriveNextTrajectoryPoint();
        pending_events_.do_evasion = false;
      }
      current_state_ = Normal;
    }
    break;
  }
}

void EGMInterfaceDefault::processCurrentTarget()
{
  motion_step_.goal.time_passed += auxiliary_.estimated_sample_time;

  switch (current_state_)
  {
    case Normal:
    case OverrideTrajectory:
    {
      // Processing Normal and OverrideTrajectory cases in the same way for both cases.
      // The target has been updated earlier according to respective case.

      motion_step_.current.references = motion_step_.goal.target;

      if (configuration_.active.simple_interpolation.use_interpolation)
      {
        simple_interpolator_.calculateReferences(motion_step_.goal.time_passed,
                                                 &motion_step_.current.references,
                                                 auxiliary_.mode);
      }
    }
    break;

    case Stop:
    case Stopping:
    {
      // Processing Stop and Stopping cases in the same way for both cases.

      simple_interpolator_.calculateReferences(motion_step_.goal.time_passed,
                                               &motion_step_.current.references,
                                               auxiliary_.mode);
    }
    break;
  }

  if (current_state_ == Stopping &&
      motion_step_.goal.time_passed > configuration_.active.simple_interpolation.ramp_down_time +
                                      auxiliary_.estimated_sample_time)
  {
    motion_step_.goal.target = motion_step_.current.references;

    if (configuration_.active.simple_interpolation.use_interpolation)
    {
      motion_step_.checkQuaternions();
      simple_interpolator_.update(motion_step_.current.references,
                                  motion_step_.goal.target,
                                  auxiliary_.mode);
    }
    motion_step_.goal.time_passed = 0.0;

    current_state_ = Stopped;
  }
}

/********************************************
 * Auxiliary methods
 */

void EGMInterfaceDefault::demoCreateEGMReferences()
{
  unsigned int seqno             = auxiliary_.sequence_number;   // Current sequence number.
  const double TS                = 0.004;                        // Sample time [s].
  const double RAMP_IN           = 3.0;                          // Ramp in time [s].
  unsigned int seqno_ramp_in_end = (unsigned int)(RAMP_IN / TS); // Sequence number when the ramp in is finished [-].
  double amplitude = 0.0;
  double a = 0.0;
  double b = 0.0;

  proto::Feedback& current_feedback  = motion_step_.current.feedback;
  proto::Feedback& initial_feedback  = auxiliary_.initial_feedback;
  proto::TrajectoryPoint& references = motion_step_.current.references;

  /*------------------------------------------------------
   * Joint space
   */
  amplitude = 2.0;
  a = amplitude / 2.0;
  b = M_PI*TS / RAMP_IN;

  double disturbance = a*cos(b*seqno) - a;
  double speed_disturbance = -(a*b)*sin(b*seqno) / TS;
  if (seqno > seqno_ramp_in_end + 1)
  {
    double freq = 2.0;
    disturbance = -2.0*a*cos(b*(seqno - seqno_ramp_in_end)*freq);
    speed_disturbance = 2.0*(a*b)*freq*sin(b*(seqno - seqno_ramp_in_end)*freq) / TS;
  }

  // Robot joints.
  for (int i = 0; i < current_feedback.joints().position_size(); ++i)
  {
    references.mutable_joints()->set_position(i, initial_feedback.joints().position().Get(i) + disturbance);
  }

  for (int i = 0; i < current_feedback.joints().speed_size(); ++i)
  {
    references.mutable_joints()->set_speed(i, speed_disturbance);
  }

  // External joints.
  for (int i = 0; i < current_feedback.joints().external_position_size(); ++i)
  {
    references.mutable_joints()->set_external_position(i, initial_feedback.joints().
                                                          external_position().Get(i) + disturbance);
  }

  for (int i = 0; i < current_feedback.joints().external_speed_size(); ++i)
  {
    references.mutable_joints()->set_external_speed(i, speed_disturbance);
  }
 
  /*------------------------------------------------------
   * Cartesian space
   */
  amplitude = 200.0;
  a = amplitude / 2.0;
  b = M_PI*TS;

  double x_disturbance = a*(cos(0.25*b*seqno + M_PI) + 1);
  double y_disturbance = x_disturbance;
  double z_disturbance = a / 2.0*(cos(0.5*b*seqno + 1.5*M_PI));

  references.mutable_cartesian()->mutable_position()->set_x(initial_feedback.cartesian().position().x() +
                                                            x_disturbance);
  references.mutable_cartesian()->mutable_position()->set_y(initial_feedback.cartesian().position().y() +
                                                            y_disturbance);
  references.mutable_cartesian()->mutable_position()->set_z(initial_feedback.cartesian().position().z() +
                                                            z_disturbance);

  double t = 0.5*(cos(0.25*b*seqno + M_PI) + 1);
  demoInterpolateQuaternions(t);

  /* Demo for control by speed only.
   *
   * Note: Need to set PosCorrGain to zero in the RAPID instruction.
   */
  //references_.mutable_cartesian()->mutable_speed()->set_value(3, 0.0);
  //references_.mutable_cartesian()->mutable_speed()->set_value(4, 4.0);
  //references_.mutable_cartesian()->mutable_speed()->set_value(5, 0.0);
}

void EGMInterfaceDefault::demoInterpolateQuaternions(double t)
{
  // Demo using Slerp (spherical linear interpolation). See https://en.wikipedia.org/wiki/Slerp for more information.

  proto::Feedback& initial_feedback  = auxiliary_.initial_feedback;
  proto::TrajectoryPoint& references = motion_step_.current.references;

  /*------------------------------------------------------
   * Retrive the data.
   */
  double q1_u0 = initial_feedback.cartesian().quaternion_orientation().u0();
  double q1_u1 = initial_feedback.cartesian().quaternion_orientation().u1();
  double q1_u2 = initial_feedback.cartesian().quaternion_orientation().u2();
  double q1_u3 = initial_feedback.cartesian().quaternion_orientation().u3();

  proto::Euler e;
  e.set_x(-90.0);
  e.set_y(0.0);
  e.set_z(180);
  proto::Quaternion q;
  eulerZYXToQuaternion(e, &q);

  if (dotProduct(q, initial_feedback.cartesian().quaternion_orientation()) < 0.0)
  {
    q.set_u0(-q.u0());
    q.set_u1(-q.u1());
    q.set_u2(-q.u2());
    q.set_u3(-q.u3());
  }

  double q2_u0 = q.u0(); // 0.0
  double q2_u1 = q.u1(); // 0.0
  double q2_u2 = q.u2(); // 1.0
  double q2_u3 = q.u3(); // 0.0

  /*------------------------------------------------------
   * Calculate Slerp.
   */
  double dot_prod = q1_u0*q2_u0 + q1_u1*q2_u1 + q1_u2*q2_u2 + q1_u3*q2_u3;

  double omega = acos(dot_prod);
  double k = 1.0 / sin(omega);

  double a = sin((1 - t)*omega)*k;
  double b = sin(t*omega)*k;

  double q3_u0 = a*q1_u0 + b*q2_u0;
  double q3_u1 = a*q1_u1 + b*q2_u1;
  double q3_u2 = a*q1_u2 + b*q2_u2;
  double q3_u3 = a*q1_u3 + b*q2_u3;

  /*------------------------------------------------------
   * Normalize and set the references.
   */
  double norm = sqrt(q3_u0*q3_u0 + q3_u1*q3_u1 + q3_u2*q3_u2 + q3_u3*q3_u3);

  references.mutable_cartesian()->mutable_quaternion_orientation()->set_u0(q3_u0 / norm);
  references.mutable_cartesian()->mutable_quaternion_orientation()->set_u1(q3_u1 / norm);
  references.mutable_cartesian()->mutable_quaternion_orientation()->set_u2(q3_u2 / norm);
  references.mutable_cartesian()->mutable_quaternion_orientation()->set_u3(q3_u3 / norm);
}

void EGMInterfaceDefault::retriveNextTrajectoryPoint()
{
  if (planned_motion_.retriveNextTrajectoryPoint(&motion_step_.goal.target))
  {
    auxiliary_.updateMode(motion_step_.goal.target.has_cartesian());
    motion_step_.validateTarget();

    if (configuration_.active.basic.use_conditions)
    {
      while (motion_step_.conditionMet(auxiliary_.mode) &&
             planned_motion_.retriveNextTrajectoryPoint(&motion_step_.goal.target))
      {
        motion_step_.validateTarget();
      }
    }

    if (configuration_.active.simple_interpolation.use_interpolation)
    {
      motion_step_.checkQuaternions();
      simple_interpolator_.update(motion_step_.current.references,
                                  motion_step_.goal.target,
                                  auxiliary_.mode);
    }
    motion_step_.goal.time_passed = 0.0;
  }
  else
  {
    planned_motion_.trajectories.p_current.reset();
  }
}

/********************************************
 * User interaction methods
 */

EGMInterfaceConfiguration EGMInterfaceDefault::getConfiguration()
{
  // Lock the interface's configuration mutex. It is released when the lock goes out of scope.
  boost::lock_guard<boost::mutex> lock(configuration_.mutex);

  return configuration_.shared;
}

void EGMInterfaceDefault::setConfiguration(const EGMInterfaceConfiguration configuration)
{
  // Lock the interface's configuration mutex. It is released when the lock goes out of scope.
  boost::lock_guard<boost::mutex> lock(configuration_.mutex);

  configuration_.shared = configuration;
  configuration_.pending_update = true;
}

void EGMInterfaceDefault::wait_for_data()
{
  direct_references_.wait_for_data();
}

void EGMInterfaceDefault::read(proto::Feedback* p_feedback, proto::RobotStatus* p_robot_status)
{
  direct_references_.read(p_feedback, p_robot_status);
}

void EGMInterfaceDefault::write(const proto::JointSpace references)
{
  direct_references_.updateTarget(references);
}

void EGMInterfaceDefault::write(const proto::CartesianSpace references)
{
  direct_references_.updateTarget(references);
}

void EGMInterfaceDefault::addTrajectory(const EGMTrajectory trajectory, const bool override_trajectories)
{
  // Lock the interface's planned motion mutex. It is released when the lock goes out of scope.
  boost::lock_guard<boost::mutex> lock(planned_motion_.trajectories.mutex);

  boost::shared_ptr<EGMTrajectory> p_trajectory(new EGMTrajectory(trajectory));

  if (override_trajectories)
  {
    planned_motion_.emptyTemporaryQueue();
    planned_motion_.queues.temporary.push(p_trajectory);
    pending_events_.do_override = override_trajectories;
  }
  else
  {
    if ((pending_events_.do_stop && pending_events_.do_discard) ||
         pending_events_.do_override)
    {
      planned_motion_.queues.temporary.push(p_trajectory);
    }
    else
    {
      planned_motion_.queues.primary.push(p_trajectory);
      ++planned_motion_.queues.primary_queue_size;
    }
  }
}

void EGMInterfaceDefault::evasiveMotion(const EGMTrajectory trajectory)
{
  // Lock the interface's planned motion mutex. It is released when the lock goes out of scope.
  boost::lock_guard<boost::mutex> lock(planned_motion_.trajectories.mutex);

  boost::shared_ptr<EGMTrajectory> p_trajectory(new EGMTrajectory(trajectory));

  pending_events_.do_stop = true;
  pending_events_.do_discard = true;
  pending_events_.do_evasion = true;

  planned_motion_.trajectories.p_evasive = p_trajectory;
}

proto::InterfaceData EGMInterfaceDefault::retriveCurrentData(bool* new_data)
{
  // Lock the interface's auxiliary data mutex. It is released when the lock goes out of scope.
  boost::lock_guard<boost::mutex> lock(auxiliary_.mutex);

  *new_data = auxiliary_.new_message;
  auxiliary_.new_message = false;

  return auxiliary_.interface_data;
}

} // end namespace egm_interface
} // end namespace abb
