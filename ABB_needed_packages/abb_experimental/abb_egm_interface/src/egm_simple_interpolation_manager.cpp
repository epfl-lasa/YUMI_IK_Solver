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

#include "abb_egm_interface/egm_common_math.h"
#include "abb_egm_interface/egm_simple_interpolation_manager.h"

namespace abb
{
namespace egm_interface
{

using namespace egm_common_values::conversions;
using namespace egm_common_values::interpolation;

/****************************************************************************************
 * Class definitions: EGMSimpleInterpolationManager
 */

#pragma region EGMSimpleInterpolationManager
/********************************************
 * Primary methods
 */

void EGMSimpleInterpolationManager::update(
     const proto::TrajectoryPoint references,
     const proto::TrajectoryPoint current_target,
     const EGMInterfaceConfiguration::InterfaceModes mode)
{
  if (current_target.has_duration() && current_target.duration() > std::numeric_limits<double>::epsilon())
  {
    // Assume that the specified duration is valid.
    t_ = current_target.duration();

    switch (mode)
    {
      case EGMInterfaceConfiguration::EGMJoint:
      {
        updateJoint(references, current_target, settings_);
      }
      break;

      case EGMInterfaceConfiguration::EGMCartesian:
      {
        updateCartesian(references, current_target, settings_);
      }
      break;
    }
  }
  else
  {
    switch (mode)
    {
      case EGMInterfaceConfiguration::EGMJoint:
      {
        attemptJoint(references, current_target, settings_);
      }
      break;

      case EGMInterfaceConfiguration::EGMCartesian:
      {
        attemptCartesian(references, current_target, settings_);
      }
      break;
    }
  }
}

void EGMSimpleInterpolationManager::update(
     const proto::TrajectoryPoint references,
     const EGMInterfaceConfiguration::InterfaceModes mode)
{
  t_ = settings_.ramp_down_time;

  if (t_ <= std::numeric_limits<double>::epsilon()) { t_ = DEFAULT_RAMP_DOWN_TIME; }

  switch (mode)
  {
    case EGMInterfaceConfiguration::EGMJoint:
    {
      updateJoint(references, references, settings_, true);
    }
    break;

    case EGMInterfaceConfiguration::EGMCartesian:
    {
      updateCartesian(references, references, settings_, true);
    }
    break;
  }
}

void EGMSimpleInterpolationManager::calculateReferences(
     double t,
     proto::TrajectoryPoint* p_references,
     const EGMInterfaceConfiguration::InterfaceModes mode)
{
  if (t > t_) { t = t_; }

  switch (mode)
  {
    case EGMInterfaceConfiguration::EGMJoint:
    {
      // Robot joints.
      for (int i = 0; i < p_references->joints().position_size(); ++i)
      {
        spline_polynomials_[i].evaluate(i, t, p_references->mutable_joints());
      }

      // External joints.
      for (int i = 0; i < p_references->joints().external_position_size(); ++i)
      {
        spline_polynomials_[i + OFFSET_].evaluate(i + OFFSET_, t, p_references->mutable_joints());
      }
    }
    break;

    case EGMInterfaceConfiguration::EGMCartesian:
    {
      // Position, speed and acceleration.
      spline_polynomials_[X_].evaluate(X_, t, p_references->mutable_cartesian());
      spline_polynomials_[Y_].evaluate(Y_, t, p_references->mutable_cartesian());
      spline_polynomials_[Z_].evaluate(Z_, t, p_references->mutable_cartesian());

      // Orientation.
      slerp_.evaluate(t/t_, p_references->mutable_cartesian());

      // External joints.
      for (int i = 0; i < p_references->joints().external_position_size(); ++i)
      {
        spline_polynomials_[i + OFFSET_].evaluate(i + OFFSET_, t, p_references->mutable_joints());
      }
    }
    break;
  }
}

/********************************************
 * Auxiliary methods
 */

void EGMSimpleInterpolationManager::updateJoint(
     const proto::TrajectoryPoint references,
     const proto::TrajectoryPoint current_target,
     EGMInterfaceConfiguration::SimpleInterpolationSettings interpolation_settings,
     const bool ramp_down)
{
  SplineBoundaryConditions conditions(interpolation_settings, ramp_down);

  // Robot joints
  for (int i = 0; i < references.joints().position_size(); ++i)
  {
    conditions.setConditions(i, references, current_target);
    spline_polynomials_[i].calculateCoefficients(t_, conditions);
  }

  // External joints
  for (int i = 0; i < references.joints().external_position_size(); ++i)
  {
    conditions.setConditions(i + OFFSET_, references, current_target);
    spline_polynomials_[i + OFFSET_].calculateCoefficients(t_, conditions);
  }
}

void EGMSimpleInterpolationManager::updateCartesian(
     const proto::TrajectoryPoint references,
     const proto::TrajectoryPoint current_target,
     EGMInterfaceConfiguration::SimpleInterpolationSettings interpolation_settings,
     const bool ramp_down)
{
  SplineBoundaryConditions conditions(interpolation_settings, ramp_down);

  // Position, speed and acceleration.
  // X.
  conditions.setConditions(X_, references, current_target);
  spline_polynomials_[X_].calculateCoefficients(t_, conditions);

  // Y.
  conditions.setConditions(Y_, references, current_target);
  spline_polynomials_[Y_].calculateCoefficients(t_, conditions);

  // Z.
  conditions.setConditions(Z_, references, current_target);
  spline_polynomials_[Z_].calculateCoefficients(t_, conditions);

  // Orientation.
  slerp_.calculateCoefficients(t_,
                               references.cartesian().quaternion_orientation(),
                               current_target.cartesian().quaternion_orientation(),
                               interpolation_settings.joint_limits);
  
  // External joints
  for (int i = 0; i < references.joints().external_position_size(); ++i)
  {
    conditions.setConditions(i + OFFSET_, references, current_target);
    spline_polynomials_[i + OFFSET_].calculateCoefficients(t_, conditions);
  }
}

void EGMSimpleInterpolationManager::attemptJoint(
     const proto::TrajectoryPoint references,
     const proto::TrajectoryPoint current_target,
     EGMInterfaceConfiguration::SimpleInterpolationSettings interpolation_settings)
{
  EGMInterfaceConfiguration::Limits joint_limits = interpolation_settings.joint_limits;

  interpolation_settings.spline_method = EGMInterfaceConfiguration::Quintic;
  SplineBoundaryConditions conditions(interpolation_settings);

  bool conditions_satisfied = true;

  t_ = 0.0;

  // Try to find a common time interval that satisfies 
  // the speed and acceleration limits.
  do
  {
    // Guess a time interval and assume that the conditions will be satisfied.
    t_ += 0.04;
    conditions_satisfied = true;

    for (int i = 0; i < references.joints().position_size() && conditions_satisfied; ++i)
    {
      conditions.setConditions(i, references, current_target);
      conditions_satisfied = spline_polynomials_[i].attemptJoint(t_, conditions, joint_limits);
    }

    for (int i = 0; i < references.joints().external_position_size() && conditions_satisfied; ++i)
    {
      conditions.setConditions(i + OFFSET_, references, current_target);
      conditions_satisfied = spline_polynomials_[i + OFFSET_].attemptJoint(t_, conditions, joint_limits);
    }
  }
  while (!conditions_satisfied && t_ < interpolation_settings.max_T);

  /*
   * If the conditions has not been satisfied or the maximum time interval has been reached,
   * use a linear interpolation instead (as "backup plan").
   */
  if (!conditions_satisfied || t_ >= interpolation_settings.max_T)
  {
    t_ = guessJointLinearTimeInterval(references, current_target, 0.5*joint_limits.speed);
        
    if (t_ <= std::numeric_limits<double>::epsilon())
    {
      t_ = interpolation_settings.max_T;
    }

    interpolation_settings.spline_method = EGMInterfaceConfiguration::Linear;
    updateJoint(references, current_target, interpolation_settings);
  }
}

void EGMSimpleInterpolationManager::attemptCartesian(
     const proto::TrajectoryPoint references,
     const proto::TrajectoryPoint current_target,
     EGMInterfaceConfiguration::SimpleInterpolationSettings interpolation_settings)
{
  EGMInterfaceConfiguration::Limits joint_limits = interpolation_settings.joint_limits;
  EGMInterfaceConfiguration::Limits cartesian_limits = interpolation_settings.cartesian_limits;

  interpolation_settings.spline_method = EGMInterfaceConfiguration::Quintic;
  SplineBoundaryConditions conditions(interpolation_settings);

  double max_speed = 0.0;
  double squared_spd = 0.0;

  double max_acceleration = 0.0;
  double squared_acc = 0.0;

  bool conditions_satisfied = true;

  t_ = 0.0;

  // Try to find a common time interval that satisfies 
  // the speed and acceleration limits.
  do
  {
    // Guess a time interval and assume that the conditions will be satisfied.
    t_ += 0.04;
    conditions_satisfied = true;

    // Position, speed and acceleration.
    // X.
    conditions.setConditions(X_, references, current_target);
    spline_polynomials_[X_].attemptCartesian(t_,
                                             conditions,
                                             &max_speed,
                                             &max_acceleration);
    squared_spd = std::pow(max_speed, 2);
    squared_acc = std::pow(max_acceleration, 2);

    // Y.
    conditions.setConditions(Y_, references, current_target);
    spline_polynomials_[Y_].attemptCartesian(t_,
                                             conditions,
                                             &max_speed,
                                             &max_acceleration);
    squared_spd += std::pow(max_speed, 2);
    squared_acc += std::pow(max_acceleration, 2);

    // Z.
    conditions.setConditions(Z_, references, current_target);
    spline_polynomials_[Z_].attemptCartesian(t_,
                                             conditions,
                                             &max_speed,
                                             &max_acceleration);
    squared_spd += std::pow(max_speed, 2);
    squared_acc += std::pow(max_acceleration, 2);

    // Check speed and acceleration limits.
    if (std::sqrt(squared_spd) > cartesian_limits.speed ||
        std::sqrt(squared_acc) > cartesian_limits.acceleration)
    {
      conditions_satisfied = false;
    }

    // Orientation.
    if (conditions_satisfied)
    {
      conditions_satisfied = slerp_.calculateCoefficients(t_,
                                                          references.cartesian().quaternion_orientation(),
                                                          current_target.cartesian().quaternion_orientation(),
                                                          joint_limits);
    }

    // External joints:
    for (int i = 0; i < references.joints().external_position_size() && conditions_satisfied; ++i)
    {
      conditions.setConditions(i + OFFSET_, references, current_target);
      conditions_satisfied = spline_polynomials_[i + OFFSET_].attemptJoint(t_, conditions, joint_limits);
    }
  }
  while (!conditions_satisfied && t_ < interpolation_settings.max_T);

  /*
   * If the conditions has not been satisfied or the maximum time interval has been reached,
   * use a linear interpolation instead (as "backup plan").
   */
  if (!conditions_satisfied || t_ >= interpolation_settings.max_T)
  {
    t_ = guessCartesianLinearTimeInterval(references,
                                          current_target,
                                          0.5*joint_limits.speed,
                                          0.5*cartesian_limits.speed);
        
    if (t_ <= std::numeric_limits<double>::epsilon())
    {
      t_ = interpolation_settings.max_T;
    }

    interpolation_settings.spline_method = EGMInterfaceConfiguration::Linear;
    updateCartesian(references, current_target, interpolation_settings);
  }
}

double EGMSimpleInterpolationManager::guessJointLinearTimeInterval(
       const proto::TrajectoryPoint references,
       const proto::TrajectoryPoint current_target,
       const double speed_limit)
{
  double max_delta = 0.0;
  double tmp_delta = 0.0;

  // Find the biggest joint difference.
  for (int i = 0; i < references.joints().position_size(); ++i)
  {
    tmp_delta = std::abs(current_target.joints().position().Get(i) - 
                         references.joints().position().Get(i));

    if (tmp_delta > max_delta) { max_delta = tmp_delta; }
  }

  // Find the biggest external joint difference.
  for (int i = 0; i < references.joints().external_position_size(); ++i)
  {
    tmp_delta = std::abs(current_target.joints().external_position().Get(i) -
                         references.joints().external_position().Get(i));

    if (tmp_delta > max_delta) { max_delta = tmp_delta; }
  }

  // Calculate a time interval T based on the biggest joint difference and the specified speed limit.
  return std::abs(max_delta) / speed_limit;
}

double EGMSimpleInterpolationManager::guessCartesianLinearTimeInterval(
       const proto::TrajectoryPoint references,
       const proto::TrajectoryPoint current_target,
       const double joint_speed_limit,
       const double cartesian_speed_limit)
{
  double tmp_T = 0.0;
  double max_T = 0.0;
  double alfa = 0.0;
  double beta = 0.0;

  // Position x, y and z.
  // X.
  alfa = references.cartesian().position().x();
  beta = current_target.cartesian().position().x();

  tmp_T = std::abs(beta - alfa)/cartesian_speed_limit;
  if (tmp_T > max_T) { max_T = tmp_T; }

  // Y.
  alfa = references.cartesian().position().y();
  beta = current_target.cartesian().position().y();

  tmp_T = std::abs(beta - alfa)/cartesian_speed_limit;
  if (tmp_T > max_T) { max_T = tmp_T; }

  // Z.
  alfa = references.cartesian().position().z();
  beta = current_target.cartesian().position().z();

  tmp_T = std::abs(beta - alfa)/cartesian_speed_limit;
  if (tmp_T > max_T) { max_T = tmp_T; }

  // Orientation.
  proto::Quaternion q1 = references.cartesian().quaternion_orientation();
  proto::Quaternion q2 = current_target.cartesian().quaternion_orientation();
  double dot_product = dotProduct(q1, q2);

  if (1.0 - abs(dot_product) < 0.0001)
  {
    dot_product = (dot_product < 0.0 ? -1.0 : 1.0);
  }

  double omega = std::acos(dot_product);

  tmp_T = std::abs(2.0*omega*RAD_TO_DEG)/joint_speed_limit;
  if (tmp_T > max_T) { max_T = tmp_T; }

  // Find the biggest time interval for the external joints.
  for (int i = 0; i < references.joints().external_position_size(); ++i)
  {
    alfa = references.joints().external_position().Get(i);
    beta = current_target.joints().external_position().Get(i);

    tmp_T = std::abs(beta - alfa)/joint_speed_limit;
    if (tmp_T > max_T) { max_T = tmp_T; } 
  }

  return max_T;
}
#pragma endregion



/****************************************************************************************
 * Struct definitions: EGMSimpleInterpolationManager::SplineConditions
 */

#pragma region EGMSimpleInterpolationManager::SplineConditions
/********************************************
 * Primary methods
 */

void EGMSimpleInterpolationManager::SplineBoundaryConditions::setConditions(
     int index,
     const proto::TrajectoryPoint references,
     const proto::TrajectoryPoint current_target)
{
  if (index < OFFSET_)
  {
    alfa_    = references.joints().position().Get(index);
    d_alfa_  = references.joints().speed().Get(index);
    dd_alfa_ = references.joints().acceleration().Get(index);
    beta_    = current_target.joints().position().Get(index);
    d_beta_  = use_speed_ ? current_target.joints().speed().Get(index) : 0.0;
    dd_beta_ = use_acceleration_ ? current_target.joints().acceleration().Get(index) : 0.0;
  }
  else
  {
    index -= OFFSET_;

    alfa_    = references.joints().external_position().Get(index);
    d_alfa_  = references.joints().external_speed().Get(index);
    dd_alfa_ = references.joints().external_acceleration().Get(index);
    beta_    = current_target.joints().external_position().Get(index);
    d_beta_  = use_speed_ ? current_target.joints().external_speed().Get(index) : 0.0;
    dd_beta_ = use_acceleration_ ? current_target.joints().external_acceleration().Get(index) : 0.0;
  }
}

void EGMSimpleInterpolationManager::SplineBoundaryConditions::setConditions(
     const EGMSimpleInterpolationManager::SplineBoundaryConditions::Axis axis,
     const proto::TrajectoryPoint references,
     const proto::TrajectoryPoint current_target)
{
  switch (axis)
  {
    case EGMSimpleInterpolationManager::SplineBoundaryConditions::X:
    {
      alfa_    = references.cartesian().position().x();
      d_alfa_  = references.cartesian().speed().value().Get(axis);
      dd_alfa_ = references.cartesian().acceleration().x();
      beta_    = current_target.cartesian().position().x();
      d_beta_  = use_speed_ ? current_target.cartesian().speed().value().Get(axis) : 0.0;
      dd_beta_ = use_acceleration_ ? current_target.cartesian().acceleration().x() : 0.0;
    }
    break;

    case EGMSimpleInterpolationManager::SplineBoundaryConditions::Y:
    {
      alfa_    = references.cartesian().position().y();
      d_alfa_  = references.cartesian().speed().value().Get(axis);
      dd_alfa_ = references.cartesian().acceleration().y();
      beta_    = current_target.cartesian().position().y();
      d_beta_  = use_speed_ ? current_target.cartesian().speed().value().Get(axis) : 0.0;
      dd_beta_ = use_acceleration_ ? current_target.cartesian().acceleration().y() : 0.0;
    }
    break;

    case EGMSimpleInterpolationManager::SplineBoundaryConditions::Z:
    {
      alfa_    = references.cartesian().position().z();
      d_alfa_  = references.cartesian().speed().value().Get(axis);
      dd_alfa_ = references.cartesian().acceleration().z();
      beta_    = current_target.cartesian().position().z();
      d_beta_  = use_speed_ ? current_target.cartesian().speed().value().Get(axis) : 0.0;
      dd_beta_ = use_acceleration_ ? current_target.cartesian().acceleration().z() : 0.0;
    }
    break;
  }
}
#pragma endregion



/****************************************************************************************
 * Class definitions: EGMSimpleInterpolationManager::SplinePolynomial
 */

#pragma region EGMSimpleInterpolationManager::SplinePolynomial
/********************************************
 * Primary methods
 */

void EGMSimpleInterpolationManager::SplinePolynomial::calculateCoefficients(
     const double T,
     const SplineBoundaryConditions conditions)
{
  double alfa = conditions.alfa_;
  double d_alfa = conditions.d_alfa_;
  double dd_alfa = conditions.dd_alfa_;
  double beta = conditions.beta_;
  double d_beta = conditions.d_beta_;
  double dd_beta = conditions.dd_beta_;
  double C1 = 0.0;
  double C2 = 0.0;
  double C3 = 0.0;

  if (conditions.ramp_down_)
  {
    /*
     * Calculate the spline polynomial coefficients for:
     * S(t) = A + B*t + C*t^2
     * 
     * Note: Used when ramping out the speed. I.e. stopping.
     *
     *---------------------
     * Conditons:
     *---------------------
     * 0 < t <= T
     *
     * S(0) = alfa
     * d_S(0) = d_alfa
     *
     * d_S(T) = 0
     */

    a_ = alfa;
    b_ = d_alfa;
    c_ = -d_alfa / (2.0 * T);
    d_ = 0.0;
    e_ = 0.0;
    f_ = 0.0;
  }
  else
  {
    switch (conditions.spline_method_)
    {
      case EGMInterfaceConfiguration::Linear:
      {
        /*
         * Calculate the spline polynomial coefficients for:
         * S(t) = A + B*t
         *
         *---------------------
         * Conditons:
         *---------------------
         * 0 < t <= T
         *
         * S(0) = alfa
         *
         * S(T) = beta
         */

        a_ = alfa;
        b_ = (beta - alfa) / T;
        c_ = 0.0;
        d_ = 0.0;
        e_ = 0.0;
        f_ = 0.0;
      }
      break;

      case EGMInterfaceConfiguration::Cubic:
      {
        /*
         * Calculate the spline polynomial coefficients for:
         * S(t) = A + B*t + C*t^2 + D*t^3
         * 
         *---------------------
         * Conditons:
         *---------------------
         * 0 < t <= T
         *
         * S(0) = alfa
         * d_S(0) = d_alfa
         *
         * S(T) = beta
         * d_S(T) = d_beta
         */

        a_ = alfa;
        b_ = d_alfa;

        C1 = beta - alfa - d_alfa*T;
        C2 = d_beta - d_alfa;

        c_ = 3.0*C1 / pow(T, 2) - C2 / T;
        d_ = C1 / pow(T, 3) - c_ / T;
        e_ = 0.0;
        f_ = 0.0;
      }
      break;

      case EGMInterfaceConfiguration::Quintic:
      {  
        /*
         * Calculate the spline polynomial coefficients for:
         * S(t) = A + B*t + C*t^2 + D*t^3 + E*t^4 + F*t^5
         *
         *---------------------
         * Conditons:
         *---------------------
         * 0 < t <= T
         *
         * S(0) = alfa
         * d_S(0) = d_alfa
         * dd_S(0) = dd_alfa
         *
         * S(T) = beta
         * d_S(T) = d_beta
         * dd_S(T) = dd_beta
         */

        a_ = alfa;
        b_ = d_alfa;
        c_ = dd_alfa / 2.0;

        C1 = beta - alfa - d_alfa*T - (dd_alfa / 2.0)*std::pow(T, 2);
        C2 = d_beta - d_alfa - dd_alfa*T;
        C3 = dd_beta - dd_alfa;

        d_ = 10.0*C1 / std::pow(T, 3) - 4.0 * C2 / std::pow(T, 2) + C3 / (2.0*T);
        e_ = 5.0*C1 / std::pow(T, 4) - C2 / std::pow(T, 3) - 2.0*d_ / T;
        f_ = C1 / std::pow(T, 5) - d_ / std::pow(T, 2) - e_ / T;
      }
      break;
    }
  }
}

bool EGMSimpleInterpolationManager::SplinePolynomial::attemptJoint(
     const double T,
     const SplineBoundaryConditions conditions,
     const EGMInterfaceConfiguration::Limits limits)
{
  bool conditions_satisfied = true;

  RootContainer roots_dd_spline;
  RootContainer roots_ddd_spline;

  double speed[RootContainer::MAX_NUMBER_OF_ROOTS];
  double acceleration[RootContainer::MAX_NUMBER_OF_ROOTS];

  for (size_t i = 0; i < RootContainer::MAX_NUMBER_OF_ROOTS; ++i)
  {
    speed[i] = 0.0;
    acceleration[i] = 0.0;
  }

  // Calculate the spline polynomial's coefficients and the roots for the first and second derivatives.
  calculateCoefficients(T, conditions);
  calculateRoots(&roots_dd_spline, &roots_ddd_spline);

  // Check the speed condition.
  for (size_t i = 0; i < roots_dd_spline.number_of_roots_ && conditions_satisfied; ++i)
  {
    // Check only real and positive roots.
    if (roots_dd_spline.roots_[i].imag() == 0.0 && roots_dd_spline.roots_[i].real() > 0.0)
    {
      speed[i] = calculateSpeed(roots_dd_spline.roots_[i].real());
      if (std::abs(speed[i]) > limits.speed)
      {
        conditions_satisfied = false;
      }
    }
  }

  // Check the acceleration condition.
  for (size_t i = 0; i < roots_ddd_spline.number_of_roots_ && conditions_satisfied; ++i)
  {
    // Check only real and positive roots.
    if (roots_ddd_spline.roots_[i].imag() == 0.0 && roots_ddd_spline.roots_[i].real() > 0.0)
    {
      acceleration[i] = calculateAcceleration(roots_ddd_spline.roots_[i].real());
      if (std::abs(acceleration[i]) > limits.acceleration)
      {
        conditions_satisfied = false;
      }
    }
  }

  return conditions_satisfied;
}

void EGMSimpleInterpolationManager::SplinePolynomial::attemptCartesian(
     const double T,
     const SplineBoundaryConditions conditions,
     double* p_max_speed,
     double* p_max_acceleration)
{
  RootContainer roots_dd_spline;
  RootContainer roots_ddd_spline;

  double tmp_speed = 0.0;;
  double max_speed = 0.0;;

  double tmp_acceleration = 0.0;
  double max_acceleration = 0.0;

  // Calculate the spline polynomial's coefficients and the roots for the first and second derivatives.
  calculateCoefficients(T, conditions);
  calculateRoots(&roots_dd_spline, &roots_ddd_spline);

  // Find max speed.
  for (size_t i = 0; i < roots_dd_spline.number_of_roots_; ++i)
  {
    // Check only real and positive roots.
    if (roots_dd_spline.roots_[i].imag() == 0.0 && roots_dd_spline.roots_[i].real() > 0.0)
    {
      tmp_speed = calculateSpeed(roots_dd_spline.roots_[i].real());
      if (std::abs(tmp_speed) > std::abs(max_speed))
      {
        max_speed = tmp_speed;
      }
    }
  }

  // Find max acceleration.
  for (size_t i = 0; i < roots_ddd_spline.number_of_roots_; ++i)
  {
    // Check only real and positive roots.
    if (roots_ddd_spline.roots_[i].imag() == 0.0 && roots_ddd_spline.roots_[i].real() > 0.0)
    {
      tmp_acceleration = calculateAcceleration(roots_ddd_spline.roots_[i].real());
      if (std::abs(tmp_acceleration) > std::abs(max_acceleration))
      {
        max_acceleration = tmp_acceleration;
      }
    }
  }

  *p_max_speed = max_speed;
  *p_max_acceleration = max_acceleration;
}

void EGMSimpleInterpolationManager::SplinePolynomial::evaluate(
     int index,
     const double t,
     proto::JointSpace* p_references)
{
  /* 
   * Calculate:
   *   S(t) = A + B*t + C*t^2 + D*t^3 + E*t^4 + F*t^5
   *   S_prime(t) = B + 2C*t + 3D*t^2 + 4E*t^3 + 5F*t^4
   *   S_bis(t) = 2C + 6D*t + 12E*t^2 + 20F*t^3
   * 
   * Condition: 0 < t <= T
   */
  double pos = calculatePosition(t);
  double spd = calculateSpeed(t);
  double acc = calculateAcceleration(t);

  if (index < OFFSET_)
  {
    p_references->mutable_position()->Set(index, pos);
    p_references->mutable_speed()->Set(index, spd);
    p_references->mutable_acceleration()->Set(index, acc);
  }
  else
  {
    index -= OFFSET_;

    p_references->mutable_external_position()->Set(index, pos);
    p_references->mutable_external_speed()->Set(index, spd);
    p_references->mutable_external_acceleration()->Set(index, acc);
  }
}

void EGMSimpleInterpolationManager::SplinePolynomial::evaluate(
     const SplineBoundaryConditions::Axis axis,
     const double t,
     proto::CartesianSpace* p_references)
{
  /* 
   * Calculate:
   *   S(t) = A + B*t + C*t^2 + D*t^3 + E*t^4 + F*t^5
   *   S_prime(t) = B + 2C*t + 3D*t^2 + 4E*t^3 + 5F*t^4
   *   S_bis(t) = 2C + 6D*t + 12E*t^2 + 20F*t^3
   * 
   * Condition: 0 < t <= T
   */
  double pos = calculatePosition(t);
  double spd = calculateSpeed(t);
  double acc = calculateAcceleration(t);

  switch (axis)
  {
    case EGMSimpleInterpolationManager::SplineBoundaryConditions::X:
    {
      p_references->mutable_position()->set_x(pos);
      p_references->mutable_speed()->set_value(axis, spd);
      p_references->mutable_acceleration()->set_x(acc);
    }
    break;

    case EGMSimpleInterpolationManager::SplineBoundaryConditions::Y:
    {
      p_references->mutable_position()->set_y(pos);
      p_references->mutable_speed()->set_value(axis, spd);
      p_references->mutable_acceleration()->set_y(acc);
    }
    break;

    case EGMSimpleInterpolationManager::SplineBoundaryConditions::Z:
    {
      p_references->mutable_position()->set_z(pos);
      p_references->mutable_speed()->set_value(axis, spd);
      p_references->mutable_acceleration()->set_z(acc);
    }
    break;
  }
}

/********************************************
 * Auxiliary methods
 */

void EGMSimpleInterpolationManager::SplinePolynomial::calculateRoots(
     RootContainer* p_roots_dd_spline,
     RootContainer* p_roots_ddd_spline)
{
  /*
   * Find the values for t that give max and min values for the speed S'(t).
   * I.e calculate the roots for: S''(t) = 2C + 6D*t + 12E*t^2 + 20F*t^3 
   */
  (*p_roots_dd_spline) = findRoots(20.0*f_, 12.0*e_, 6.0*d_, 2.0*c_);

  /*
   * Find the values for t that give max and min values for the acceleration S''(t).
   * I.e calculate the roots for: S'''(t) = 6D + 24E*t + 60F*t^2
   */
  (*p_roots_ddd_spline) = findRoots(0.0, 60.0*f_, 24.0*e_, 6.0*d_);
}

EGMSimpleInterpolationManager::SplinePolynomial::RootContainer
EGMSimpleInterpolationManager::SplinePolynomial::findRoots(const double a,
                                                           const double b,
                                                           const double c,
                                                           const double d)
{
  // Find the roots for a general polynomial equation of degree 3 or lower. I.e. a*t^3 + b*t^2 + c*t + d = 0

  RootContainer roots;
  double discriminant;

  if (a != 0.0)
  {
    // Case: cubic (see https://en.wikipedia.org/wiki/Cubic_function for more information on how to calculate the roots)
    roots.number_of_roots_ = 3;

    discriminant = 18*a*b*c*d -
                   4*std::pow(b, 3)*d +
                   std::pow(b, 2)*std::pow(c, 2) -
                   4*a*std::pow(c, 3) -
                   27*std::pow(a, 2)*std::pow(d, 2);

    double delta0 = std::pow(b, 2) - 3 * a*c;
    double delta1 = 2 * std::pow(b, 3) - 9 * a*b*c + 27 * std::pow(a, 2)*d;

    if (discriminant == 0)
    {
      if (delta0 == 0)
      {
        roots.roots_[0] = roots.roots_[1] = roots.roots_[2] = -b / (3.0*a);
      }
      else
      {
        roots.roots_[0] = roots.roots_[1] = (9 * a*d - b*c) / (2 * delta0);
        roots.roots_[2] = (4 * a*b*c - 9 * std::pow(a, 2)*d - std::pow(b, 3)) / (a*delta0);
      }
    }
    else
    {
      std::complex<double> temp;
      if (delta0 == 0)
      {
        temp = std::complex<double>(delta1, 0.0);
      }
      else
      {
        temp = std::sqrt(std::complex<double>(std::pow(delta1, 2) - 4.0*std::pow(delta0, 3), 0.0));
      }

      std::complex<double> C_ = std::pow(((delta1 + temp) / 2.0), 1.0 / 3.0);

      boost::array<std::complex<double>, RootContainer::MAX_NUMBER_OF_ROOTS> u;

      u[0] = 1.0;
      u[1] = std::complex<double>(-1.0 / 2.0, std::sqrt(3.0) / 2.0);
      u[2] = std::complex<double>(-1.0 / 2.0, -std::sqrt(3.0) / 2.0);

      for (size_t i = 0; i < roots.number_of_roots_; ++i)
      {
        roots.roots_[i] = -1.0 / (3.0*a)*(b + u[i] * C_ + delta0 / (u[i] * C_));

        if (std::abs(roots.roots_[i].imag()) < std::pow(1.0, -15.0))
        {
          roots.roots_[i].imag(0.0);
        }
      }
    }
  }
  else if (b != 0.0)
  {
    // Case: quadratic
    roots.number_of_roots_ = 2;

    discriminant = std::pow(c, 2) - 4 * b*d;

    if (discriminant == 0.0)
    {
      roots.roots_[0] = roots.roots_[1] = -b / (2.0*c);
    }
    else
    {
      roots.roots_[0] = (-c + std::sqrt(std::complex<double>(discriminant))) / (2.0*b);
      roots.roots_[1] = (-c - std::sqrt(std::complex<double>(discriminant))) / (2.0*b);
    }
  }
  else if (c != 0.0)
  {
    // Case: linear
    roots.number_of_roots_ = 1;

    roots.roots_[0] = -d / c;
  }
  else
  {
    // Case: constant
    roots.number_of_roots_ = 0;
  }

  return roots;
}
#pragma endregion



/****************************************************************************************
 * Class definitions: EGMSimpleInterpolationManager::Slerp
 */

#pragma region EGMSimpleInterpolationManager::Slerp
/********************************************
 * Primary methods
 */

bool EGMSimpleInterpolationManager::Slerp::calculateCoefficients(
     const double T,
     const proto::Quaternion q1,
     const proto::Quaternion q2,
     const EGMInterfaceConfiguration::Limits limits)
{
  q1_ = q1;
  q2_ = q2;

  bool conditions_satisfied = true;
  double dot_product = dotProduct(q1, q2);

  if (1.0 - abs(dot_product) < 0.0001)
  {
    dot_product = (dot_product < 0.0 ? -1.0 : 1.0);
  }

  double omega = std::acos(dot_product);

  if (2.0*omega*RAD_TO_DEG / T > limits.speed)
  {
    // Leads to linear interpolation later on.
    omega_ = 0.0;

    conditions_satisfied = false;
  }
  else
  {
    omega_ = omega;
    k_ = 1.0 / std::sin(omega_);
  }

  return conditions_satisfied;
}

void EGMSimpleInterpolationManager::Slerp::evaluate(const double t, proto::CartesianSpace* q)
{
  double A = 1.0;
  double B = 0.0;

  if (omega_ < std::numeric_limits<double>::epsilon())
  {
    // Calculate quaternion with linear interpolation.
    A = 1 - t;
    B = t;
  }
  else
  {
    // Calculate quaternion with Slerp interpolation.
    A = std::sin((1 - t)*omega_)*k_;
    B = std::sin(t*omega_)*k_;
  }

  // Calculate the quaternion.
  q->mutable_quaternion_orientation()->set_u0(A*q1_.u0() + B*q2_.u0());
  q->mutable_quaternion_orientation()->set_u1(A*q1_.u1() + B*q2_.u1());
  q->mutable_quaternion_orientation()->set_u2(A*q1_.u2() + B*q2_.u2());
  q->mutable_quaternion_orientation()->set_u3(A*q1_.u3() + B*q2_.u3());

  // Normalize the quaternion.
  normalizeQuaternion(q->mutable_quaternion_orientation());
}
#pragma endregion

} // end namespace egm_interface
} // end namespace abb
