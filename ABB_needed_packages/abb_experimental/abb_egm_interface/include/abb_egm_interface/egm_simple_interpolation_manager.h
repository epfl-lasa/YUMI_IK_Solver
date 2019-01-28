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

#ifndef EGM_SIMPLE_INTERPOLATION_MANAGER_H
#define EGM_SIMPLE_INTERPOLATION_MANAGER_H

#include <complex>

#include <boost/array.hpp>

#include "egm_common.h"

namespace abb
{
namespace egm_interface
{
/**
 * \brief A class for managing simple interpolation.
 *
 * Warning:
 * - For joint mode: 
 * - 5th (or lower) degree spline polynomials are used between two points in a trajectory
 * - and NO kinematics are considered. I.e. joint limits can be exceeded (which can be a problem).
 *
 * - For Cartesian mode:
 * - TCP position uses 5th (or lower) degree spline polynomials and TCP orientation uses Slerp.
 * - Otherwise it will be similar to the joint mode interpolation.
 *
 * Improvements: 
 * Would be better to have kinematics and maybe interpolate over the whole trajectory.
 * However, at the moment, that is out of scope for this simple interpolation approach.
 */
class EGMSimpleInterpolationManager
{
public:
  /**
   * \brief A constructor.
   *
   * \param settings for the interpolation manager's interpolation settings.
   */
  EGMSimpleInterpolationManager(const EGMInterfaceConfiguration::SimpleInterpolationSettings settings)
  :
  settings_(settings),
  t_(0.0)
  {}

  /**
   * \brief A destructor.
   */
  ~EGMSimpleInterpolationManager() {}

  /**
   * \brief A method for updating the interpolation objects.
   *
   * E.g. used after a new point in a trajectory has been activated.
   *
   * \param references containing the current references (position, speed and acceleration).
   * \param current_target containing the current target (position, speed and acceleration).
   * \param mode for the interface's mode.
   */
  void update(const proto::TrajectoryPoint references,
              const proto::TrajectoryPoint current_target,
              const EGMInterfaceConfiguration::InterfaceModes mode);

  /**
   * \brief A method for updating the interpolation objects.
   *
   * E.g. used for ramping down the speed to zero.
   * 
   * \param references containing the current references (position and speed and acceleration).
   * \param mode for the interface's mode.
   */
  void update(const proto::TrajectoryPoint references,
              const EGMInterfaceConfiguration::InterfaceModes mode);

  /**
   * \brief Updates the interpolation manager's settings.
   *
   * \param settings for the interpolation manager's interpolation settings.
   */
  void updateSettings(const EGMInterfaceConfiguration::SimpleInterpolationSettings settings)
  {
    settings_ = settings;
  }

  /**
   * \brief A method for calculating the reference values.
   * 
   * \param t for the time instance (in the interval 0 < t <= T) that the interpolation should be calculated at.
   * \param p_references for storing the calculated position, speed and acceleration references.
   * \param mode for the interface's mode.
   */
  void calculateReferences(double t,
                           proto::TrajectoryPoint* p_references,
                           const EGMInterfaceConfiguration::InterfaceModes mode);

  /**
   * \brief A method for retriving the currently calculated time interval.
   * 
   * \return double containing the time interval.
   */
  double getT()
  {
    return t_;
  }

private:
  /**
   * \brief A struct for containing the boundary conditions for a spline polynomial.
   */
  struct SplineBoundaryConditions
  { 
    /**
     * \brief An enum for specifying which Cartesian axis to consider.
     */
    enum Axis
    {
      X, ///< Cartesian x axis.
      Y, ///< Cartesian y axis.
      Z  ///< Cartesian z axis.
    };

    /**
     * \brief A constructor.
     *
     * \param interpolation_settings for the interface's simple interpolation configuration.
     * \param ramp_down flag indicating if ramp down splines should be used or not.
     */
    SplineBoundaryConditions(const EGMInterfaceConfiguration::SimpleInterpolationSettings interpolation_settings,
                             const bool ramp_down = false)
      :
      alfa_(0.0),
      d_alfa_(0.0),
      dd_alfa_(0.0),
      beta_(0.0),
      d_beta_(0.0),
      dd_beta_(0.0)
    {
      ramp_down_ = ramp_down;

      spline_method_ = interpolation_settings.spline_method;
      use_speed_ = interpolation_settings.use_speed;
      use_acceleration_ = interpolation_settings.use_acceleration;
    }

    /**
     * \brief A destructor.
     */
    ~SplineBoundaryConditions() {}

    /**
     * \brief A method for extracting and setting the boundary conditions condtions for joint interpolation.
     *
     * \param index of the joint.
     * \param references containing the current references (position, speed and acceleration).
     * \param current_target containing the current target (position, speed and acceleration).
     */
    void setConditions(int index,
                       const proto::TrajectoryPoint references,
                       const proto::TrajectoryPoint current_target);

    /**
     * \brief A method for extracting and setting the boundary conditions for Cartesian interpolation.
     *
     * \param axis specifying the axis to consider.
     * \param references containing the current references (position, speed and acceleration).
     * \param current_target containing the current target (position, speed and acceleration).
     */
    void setConditions(const Axis axis,
                       const proto::TrajectoryPoint references,
                       const proto::TrajectoryPoint current_target);

    /**
     * \brief Flag indicating if ramp down splines should be used.
     */
    bool ramp_down_;

    /**
     * \brief Specifies which spline method to use.
     */
    EGMInterfaceConfiguration::SplineMethod spline_method_;

    /**
     * \brief Flag indicating if speed input values should be used in the interpolation.
     */
    bool use_speed_;

    /**
     * \brief Flag indicating if acceleration input values should be used in the interpolation.
     */
    bool use_acceleration_;

    /**
     * \brief The initial position.
     */
    double alfa_;

    /**
     * \brief The initial speed.
     */
    double d_alfa_;

    /**
     * \brief The initial acceleration.
     */
    double dd_alfa_;

    /**
     * \param The end position.
     */
    double beta_;

    /**
     * \param The end speed.
     */
    double d_beta_;

    /**
     * \param The end acceleration.
     */
    double dd_beta_;
  };

  /**
   * \brief A class for a spline interpolation polynomial of degree 5 or lower.
   * 
   * I.e. A + B*t + C*t^2 + D*t^3 + E*t^4 + F*t^5.
   */
  class SplinePolynomial
  {
  public:
    /**
     * \brief A struct for containing roots for polynomials of degree 3 or lower.
     */
    struct RootContainer
    {
      /**
       * \brief Maximum number of roots.
       */
      static const size_t MAX_NUMBER_OF_ROOTS = 3;

      /**
       * \brief Number of valid roots.
       */
      unsigned int number_of_roots_;

      /**
       * \brief Container for all the roots.
       */
      boost::array<std::complex<double>, MAX_NUMBER_OF_ROOTS> roots_;
    };

    /**
     * \brief A default constructor.
     */
    SplinePolynomial() : a_(0.0), b_(0.0), c_(0.0), d_(0.0), e_(0.0), f_(0.0) {}

    /**
     * \brief A destructor.
     */
    ~SplinePolynomial() {}

    /**
     * \brief A method for calculating the polynomial's coefficients.
     *
     * \param T for when the interpolation should reach the target.
     * \param conditions containing the boundary conditions.
     */
    void calculateCoefficients(const double T,
                               const SplineBoundaryConditions conditions);

    /**
     * \brief A method for attempting qunitic polynomials for joint mode and checking if the limits are satisfied.
     *
     * \param T spline's time inverval.
     * \param conditions containing the boundary conditions.
     * \param limits specifing the polynomial's max speed and max acceleration.
     *
     * \return bool indicating if the calculated polynomial passed the conditions or not.
     */
    bool attemptJoint(const double T,
                      SplineBoundaryConditions conditions,
                      const EGMInterfaceConfiguration::Limits limits);

    /**
     * \brief A method for attempting qunitic polynomials for Cartesian mode.
     *
     * \param T spline's time inverval.
     * \param conditions containing the boundary conditions.
     * \param p_max_speed containing the polynomial's max speed.
     * \param p_max_acceleration containing the polynomial's max acceleration.
     */
    void attemptCartesian(const double T,
                          SplineBoundaryConditions conditions,
                          double* p_max_speed,
                          double* p_max_acceleration);

    /**
     * \brief A method for evaluating the polynomial.
     *
     * \param index of the joint.
     * \param t for the time instance to evaluate.
     * \param p_references for storing the values evaluated at the time instance t.
     */
    void evaluate(int index,
                  const double t,
                  proto::JointSpace* p_references);

    /**
     * \brief A method for evaluating the polynomial.
     *
     * \param axis specifying the axis to consider.
     * \param t for the time instance to evaluate.
     * \param p_references for storing the values evaluated at the time instance t.
     */
    void evaluate(const SplineBoundaryConditions::Axis axis,
                  const double t,
                  proto::CartesianSpace* p_references);

  private:
    /**
     * \brief A method for calculating the position.
     *
     * \param t for the time instance to calculate at.
     *
     * \return double containing the calculated position.
     */
    double calculatePosition(const double t)
    {
      return a_ + b_*t + c_*std::pow(t, 2) + d_*std::pow(t, 3) + e_*std::pow(t, 4) + f_*std::pow(t, 5);
    }

    /**
     * \brief A method for calculating the speed.
     *
     * \param t for the time instance to calculate at.
     *
     * \return double containing the calculated speed.
     */
    double calculateSpeed(const double t)
    {
      return b_ + 2.0*c_*t + 3.0*d_*std::pow(t, 2) + 4.0*e_*std::pow(t, 3) + 5.0*f_*std::pow(t, 4);
    }

    /**
     * \brief A method for calculating the acceleration.
     *
     * \param t for the time instance to calculate at.
     *
     * \return double containing the calculated acceleration.
     */
    double calculateAcceleration(const double t)
    {
      return 2.0*c_ + 6.0*d_*t + 12.0*e_*std::pow(t, 2) + 20.0*f_*std::pow(t, 3);
    }

    /**
     * \brief A method for calculating the polynomial's second and third derivates' roots.
     *
     * \param p_roots_dd_spline for storing the second derivative's roots. 
     * \param p_roots_ddd_spline for storing the third derivative's roots. 
     */
    void calculateRoots(RootContainer* p_roots_dd_spline,
                        RootContainer* p_roots_ddd_spline);

    /**
     * \brief A method for finding the roots of a polynomial of degree three or lower.
     *
     * I.e. a*t^3 + b*t^2 + c*t + d = 0.
     *
     * \param a for coefficient to t^3. 
     * \param b for coefficient to t^2. 
     * \param c for coefficient to t^1.
     * \param d for coefficient to t^0.
     *
     * \return RootContainer containing roots for a polynomial of degree three or lower.
     */
    RootContainer findRoots(const double a,
                            const double b,
                            const double c,
                            const double d);

    /**
     * \brief Coefficient A.
     */
    double a_;

    /**
     * \brief Coefficient B.
     */
    double b_;

    /**
     * \brief Coefficient C.
     */
    double c_;

    /**
     * \brief Coefficient D.
     */
    double d_;

    /**
     * \brief Coefficient E.
     */
    double e_;

    /**
     * \brief Coefficient F.
     */
    double f_;
  };
  
  /**
   * \brief A class for a Slerp (Spherical linear interpolation) for quaternion interpolation.
   *        Slerp with unit quaternions produce a roataion with uniform angular velocity. 
   *
   * I.e. Slerp(q1, q2; t) = [sin((1-t)*omega)/sin(omega)]*q1 + [sin(t*omega)/sin(omega)]*q2.
   *      Where q1 and q2 are quaternions and cos(omega) = q1*q2 (dot product).
   *
   * See https://en.wikipedia.org/wiki/Slerp for more information.
   */
  class Slerp
  {
  public:
    /**
     * \brief A default constructor.
     */
    Slerp() : omega_(0.0), k_(0.0)
    {
      q1_.set_u0(1.0);
      q1_.set_u1(0.0);
      q1_.set_u2(0.0);
      q1_.set_u3(0.0);

      q2_.set_u0(1.0);
      q2_.set_u1(0.0);
      q2_.set_u2(0.0);
      q2_.set_u3(0.0);
    }

    /**
     * \brief A destructor.
     */
    ~Slerp() {}

    /**
     * \brief A method for calculating the Slerp's coefficients.
     *
     * \param T for when the interpolation should reach the target.
     * \param q1 for the initial quaternion.
     * \param q2 for the end quaternion.
     * \param limits specifing the Slerp's max speed.
     *
     * \return bool indicating if the calculated Slerp passed the conditions or not.
     */
    bool calculateCoefficients(const double T,
                               const proto::Quaternion q1,
                               const proto::Quaternion q2,
                               const EGMInterfaceConfiguration::Limits limits);

    /**
     * \brief A method for evaluating the Slerp.
     *
     * \param t for the time instance to evaluate.
     * \param p_references for storing the values evaluated at the time instance t.
     */
    void evaluate(const double t, proto::CartesianSpace* p_references);

  private:
    /**
     * \brief Initial quaternion.
     */
    proto::Quaternion q1_;

    /**
     * \brief End quaternion.
     */
    proto::Quaternion q2_;

    /**
     * \brief Coefficient omega.
     */
    double omega_;

    /**
     * \brief Coefficient K.
     */
    double k_;
  };

  /**
   * \brief A method for updating the joint mode interpolation objects.
   *
   * \param references containing the current references (position, speed and acceleration).
   * \param current_target containing the current target (position, speed and acceleration).
   * \param interpolation_settings for the server's simple interpolation configuration.
   * \param ramp_down flag indicating if ramp down spline should be used or not.
   */
  void updateJoint(const proto::TrajectoryPoint references,
                   const proto::TrajectoryPoint current_target,
                   EGMInterfaceConfiguration::SimpleInterpolationSettings interpolation_settings,
                   const bool ramp_down = false);

  /**
   * \brief A method for updating the Cartesian mode interpolation objects.
   *
   * \param references containing the current references (position, speed and acceleration).
   * \param current_target containing the current target (position, speed and acceleration).
   * \param interpolation_settings for the server's simple interpolation configuration.
   * \param ramp_down flag indicating if ramp down spline should be used or not.
   */
  void updateCartesian(const proto::TrajectoryPoint references,
                       const proto::TrajectoryPoint current_target,
                       EGMInterfaceConfiguration::SimpleInterpolationSettings interpolation_settings,
                       const bool ramp_down = false);

  /**
   * \brief A method that attempts to find a time interval, for a qunitic spline polynomial, which satisfies
   *        the specified conditions. If no solution is found, then a linear spline polynomial is used.
   *
   * I.e. used if the duration has NOT been specified in the targeted point (not the recommended way).
   *
   * \param references containing the current references (position, speed and acceleration).
   * \param current_target containing the current target (position, speed and possibly acceleration).
   * \param interpolation_settings for the server's simple interpolation configuration.
   */
  void attemptJoint(const proto::TrajectoryPoint references,
                    const proto::TrajectoryPoint current_target,
                    EGMInterfaceConfiguration::SimpleInterpolationSettings interpolation_settings);

  /**
   * \brief A method that attempts to find a time interval, for a qunitic spline polynomial and a Slerp
   *        (Spherical linear interpolation), which satisfies the specified conditions. 
   *        If no solution is found, then a linear spline polynomial is used for the position.
   *
   * I.e. used if the duration has NOT been specified in the targeted point (not the recommended way).
   *
   * \param references containing the current references (position, speed and acceleration).
   * \param current_target containing the current target (position, speed and possibly acceleration).
   * \param interpolation_settings for the server's simple interpolation configuration.
   */
  void attemptCartesian(const proto::TrajectoryPoint references,
                        const proto::TrajectoryPoint current_target,
                        const EGMInterfaceConfiguration::SimpleInterpolationSettings interpolation_settings);

  /**
   * \brief A method for guessing a time interval to be used with a linear backup polynomial (for joint mode).
   *
   * I.e. used if the duration has NOT been specified in the targeted point (not the recommended way) and
   * the attempt to find a qunitic solution FAILED.
   *
   * \param references containing the current references (position, speed and acceleration).
   * \param current_target containing the current target (position, speed and possibly acceleration).
   * \param speed_limit for the specified speed limit.
   *
   * \return double value with the guess.
   */
  double guessJointLinearTimeInterval(const proto::TrajectoryPoint references,
                                      const proto::TrajectoryPoint current_target,
                                      const double speed_limit);

  /**
   * \brief A method for guessing a time interval to be used with a linear backup interpolation (for Cartesian mode).
   *
   * I.e. used if the duration has NOT been specified in the targeted point (not the recommended way) and
   * the attempt to find a qunitic solution FAILED.
   *
   * \param references containing the current references (position, speed and acceleration).
   * \param current_target containing the current target (position, speed and possibly acceleration).
   * \param joint_speed_limit for the specified joint speed limit.
   * \param cartesian_speed_limit for the specified Cartesian speed limit.
   *
   * \return double value with the guess.
   */
  double guessCartesianLinearTimeInterval(const proto::TrajectoryPoint references,
                                          const proto::TrajectoryPoint current_target,
                                          const double joint_speed_limit,
                                          const double cartesian_speed_limit);

  /**
   * \brief Static constant for the offset in the spline polynomial array to the external joints elements.
   */
  static const size_t OFFSET_ = egm_common_values::robot_controller::DEFAULT_NUMBER_OF_ROBOT_JOINTS;

  /**
   * \brief Static constant for the x axis.
   */
  static const SplineBoundaryConditions::Axis X_ = SplineBoundaryConditions::X;

  /**
   * \brief Static constant for the y axis.
   */
  static const SplineBoundaryConditions::Axis Y_ = SplineBoundaryConditions::Y;

  /**
   * \brief Static constant for the z axis.
   */
  static const SplineBoundaryConditions::Axis Z_ = SplineBoundaryConditions::Z;

  /**
   * \brief A container for the spline interpolation polynomials.
   */
  boost::array<SplinePolynomial, egm_common_values::robot_controller::MAX_NUMBER_OF_JOINTS> spline_polynomials_;

  /**
   * \brief A container for the Slerp (for interpolating quaterions).
   */
  Slerp slerp_;

  /**
   * \brief The time interval the interpolations are valid.
   */
  double t_;

  /**
   * \brief The interpolation settings.
   */
  EGMInterfaceConfiguration::SimpleInterpolationSettings settings_;
};

} // end namespace egm_interface
} // end namespace abb

#endif // EGM_SIMPLE_INTERPOLATION_MANAGER_H
