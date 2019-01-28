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

#ifndef EGM_INTERFACE_DEFAULT_H
#define EGM_INTERFACE_DEFAULT_H

#include <boost/atomic.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/thread.hpp>

#include "egm_common.h"
#include "egm_message_manager.h"
#include "egm_server.h"
#include "egm_simple_interpolation_manager.h"

namespace abb
{
namespace egm_interface
{
/**
 * \brief A class for processing asynchronous callbacks.
 *
 * A class providing default behavior for processing asynchronous callbacks from an EGM server.
 * The interface object recieves the robot controller's outbound messages and construct inbound
 * messages to the robot controller.
 */
class EGMInterfaceDefault : public AbstractEGMInterface
{
public:
  /**
   * \brief A constructor.
   *
   * \param io_service for operating boost asio's asynchronous functions.
   * \param port_number for the server's UDP socket.
   * \param initial_configuration for the interface's configuration.
   */
  EGMInterfaceDefault(boost::asio::io_service& io_service,
                      const size_t port_number,
                      const EGMInterfaceConfiguration initial_configuration = EGMInterfaceConfiguration());

  /**
   * \brief A destructor.
   */
  ~EGMInterfaceDefault();

  /**
  * \brief Retrive the current configuration of the interface.
  *
  * \return the current configuration.
  */
  EGMInterfaceConfiguration getConfiguration();

  /**
  * \brief Update the interface's configuration (update is only applied for new sessions).
  *
  * \param configuration for the updated configuration.
  */
  void setConfiguration(const EGMInterfaceConfiguration configuration);

  /**
   * \brief Method for waiting on the next EGM message.
   *
   * I.e. blocking until a condition variable is notified (when a new EGM message has been received).
   *
   * IMPORTANT: For use with EGMInterfaceConfiguration::ExecutionModes::Direct.
   */
  void wait_for_data();
  
  /**
   * \brief Method for reading feedback and robot status.
   *
   * IMPORTANT: For use with EGMInterfaceConfiguration::ExecutionModes::Direct.
   *
   * \param p_feedback for containing the feedback.
   * \param p_robot_status for containing the robot status.
   */
  void read(proto::Feedback* p_feedback, proto::RobotStatus* p_robot_status);
  
  /**
   * \brief Method for writing joint space references.
   *
   * IMPORTANT: For use with EGMInterfaceConfiguration::ExecutionModes::Direct.
   *
   * \param references containing the joint space references.
   */
  void write(proto::JointSpace references);
  
  /**
   * \brief Method for writing Cartesian space references.
   *
   * IMPORTANT: For use with EGMInterfaceConfiguration::ExecutionModes::Direct.
   *
   * \param references containing the Cartesian space references.
   */
  void write(proto::CartesianSpace references);

  /**
   * \brief Method for adding a trajectory that the robot should follow.
   *
   * IMPORTANT: For use with EGMInterfaceConfiguration::ExecutionModes::Trajectory.
   *
   * \param trajectory containing the trajectory to add.
   * \param override_trajectories flag for if the new trajectory should override all current valid trajectories.
   */
  void addTrajectory(const EGMTrajectory trajectory, const bool override_trajectories = false);

  /**
   * \brief Method for retriving data to publish outside the interface.
   *
   * I.e. the planned trajectory and the robot's current position and status.
   *
   * IMPORTANT: For use with EGMInterfaceConfiguration::ExecutionModes::Trajectory.
   *
   * \param new_data indicating if the data has been updated since last time.
   *
   * \return proto::InterfaceData containing the data to publish outside the interface.
   */
  proto::InterfaceData retriveCurrentData(bool* new_data);

  /**
   * \brief Method for making an evasive motion.
   *
   * I.e. discarding current trajectories, ramping out the speed and going for a new target.
   *
   * IMPORTANT: For use with EGMInterfaceConfiguration::ExecutionModes::Trajectory.
   *
   * \param trajectory with the new target to go to after the speed has been ramped out.
   */
  void evasiveMotion(const EGMTrajectory trajectory);

protected:
  struct MotionStep;
  struct PlannedMotion;

  /**
   * \brief An enum for the different states the interface can handle.
   */
  enum InterfaceState
  {
    Normal,             ///< \brief Get the current target from the current trajectory.
    OverrideTrajectory, ///< \brief Discard the current target and trajectories. Then replace them with a new trajecotry.
    Stop,               ///< \brief Discard the current target and trajectories. Then ramp down the current speed to zero.
    Stopping,           ///< \brief Ramping down the current speed to zero.
    Stopped             ///< \brief The speed has been ramped down to zero.
  };

  /**
   * \brief Struct for containing auxiliary data.
   */
  struct Auxiliary
  {
    /**
     * \brief Default constructor.
     */
    Auxiliary();

    /**
     * \brief Initialize the data for a new communication session.
     *
     * \param p_egm_messages containing the recieved message.
     */
    void initializeData(EGMMessageManager* p_egm_messages);

    /**
     * \brief Calculates how long logging has occured.
     *
     * \return double for the logged amount of time.
     */
    double loggedTime() {return number_of_logged_messages*estimated_sample_time;}

    /**
     * \brief Updates the used mode (determined from user input).
     *
     * \param use_cartesian indicating that Cartesian mode should be used.
     */
    void updateMode(bool use_cartesian)
    {
      mode = (use_cartesian ? EGMInterfaceConfiguration::EGMCartesian : 
                              EGMInterfaceConfiguration::EGMJoint);
    }

    /**
    * \brief Update the data to publish outside the server.
    *
    * \param motion_step Containing data for the current motion.
    * \param planned_motion Containing data for the planned motion.
    */
    void updateInterfaceData(const MotionStep& motion_step, const PlannedMotion& planned_motion);

    /**
    * \brief Update the speed data to publish outside the server.
    *
    * \param motion_step Containing data for the current motion.
    */
    void updateInterfaceDataSpeed(const MotionStep& motion_step);

    /**
     * \brief Estimated sample time, calculated from current and previous timestamps.
     */
    double estimated_sample_time;

    /**
     * \brief A flag indicating if it is the first call of the callback method.
     */
    bool first_call;

    /**
     * \brief Container for the initial robot feedback values (used in the demo methods).
     */
    proto::Feedback initial_feedback;

    /**
     * \brief The currently used mode, e.g. joint or Cartesian (determined from user input).
     */
    EGMInterfaceConfiguration::InterfaceModes mode;

    /**
     * \brief Number of logged messages.
     */
    unsigned int number_of_logged_messages;

    /**
     * \brief The current reply message's sequence number.
     */
    unsigned int sequence_number;

    /**
    * \param Container for data to publish outside the interface.
    */
    proto::InterfaceData interface_data;

    /**
    * \brief A flag indicating if the EGM interface has recived new a message, from a robot controller,
    *        since the last retrival of interface data (from the application using the interface).
    */
    boost::atomic<bool> new_message;

    /**
    * \brief A mutex for protecting the interface's auxiliary data.
    */
    boost::mutex mutex;
  };

  /**
   * \brief Struct for containing the configuration data.
   */
  struct Configuration
  {
    /**
     * \brief A constructor.
     */
    Configuration(EGMInterfaceConfiguration initial_configuration);

    /**
     * \brief Active configuration.
     */
    EGMInterfaceConfiguration active;

    /**
     * \brief A mutex for protecting the interface's configuration.
     */
    boost::mutex mutex;

    /**
     * \brief Flag indicating if the active configuration should be updated.
     */
    boost::atomic<bool> pending_update;

    /**
     * \brief Shared configuration.
     */
    EGMInterfaceConfiguration shared;
  };

  /**
  * \brief Struct for managing direct references.
  */
  struct DirectReferences
  {
    /**
     * \brief A constructor.
     */
    DirectReferences();

    /**
     * \brief Initialize the data for a new communication session.
     */
    void initializeData();

    /**
     * \brief Wait on the next EGM message.
     */
    void wait_for_data();
    
    /**
     * \brief Read feedback and robot status.
     *
     * \param p_feedback for containing the feedback.
     * \param p_robot_status for containing the robot status.
     */
    void read(proto::Feedback* p_feedback, proto::RobotStatus* p_robot_status);
    
    /**
     * \brief Update the current direct target with joint space data.
     *
     * \param references containing the joint space references.
     */
    void updateTarget(proto::JointSpace references);
    
    /**
     * \brief Update the current direct target with Cartesian space data.
     *
     * \param references containing the Cartesian space references.
     */
    void updateTarget(proto::CartesianSpace references);
    
    /**
     * \brief Update the interface's current references.
     *
     * \param p_references for containing the new direct reference values.
     */
    void updateReferences(proto::TrajectoryPoint* p_references);
    
    /**
     * \brief Container for the direct target.
     */
    proto::TrajectoryPoint target;
    
    /**
     * \brief Mutex for protecting read data.
     */
    boost::mutex read_mutex;

    /**
     * \brief Mutex for protecting write data.
     */
    boost::mutex write_mutex;
    
    /**
     * \brief Condition variable for waiting on read data.
     */
    boost::condition_variable read_condition_variable;
    
    /**
     * \brief Condition variable for waiting on write data.
     */
    boost::condition_variable write_condition_variable;

    /**
     * \brief Flag indicating if read data is ready.
     */
    boost::atomic<bool> read_data_ready;

    /**
     * \brief Flag indicating if write data is ready.
     */
    boost::atomic<bool> write_data_ready;

    /**
     * \brief Container for the feedback.
     */
    proto::Feedback feedback;
    
    /**
     * \brief Container for the robot status.
     */
    proto::RobotStatus robot_status;
  };

  /**
   * \brief Struct for containing motion step data. 
   */
  struct MotionStep
  {
    /**
     * \brief Struct for containing data about the current motion step. 
     */
    struct CurrentStep
    {
      /**
       * \brief Default constructor.
       */
      CurrentStep() : timestamp(0) {};

      /**
       * \brief Container for the current robot feedback values.
       */
      proto::Feedback feedback;

      /**
       * \brief Container for the current robot planned values.
       */
      proto::Planned planned;

      /**
       * \brief The calculated values to use as references in the EGM reply message.
       */
      proto::TrajectoryPoint references;

      /**
       * \brief Current robot message's timestamp.
       */
      unsigned int timestamp;
    };

    /**
     * \brief Struct for containing data about the current goal. 
     */
    struct Goal
    {
      /**
       * \brief Default constructor.
       */
      Goal() : time_passed(0.0) {};

      /**
       * \brief The current targeted point.
       */
      proto::TrajectoryPoint target;

      /**
       * \brief Estimated time passed after a new target has been selected.
       */
      double time_passed;
    };

    /**
     * \brief Struct for containing data about the previous motion step. 
     */
    struct PreviousStep
    {
      /**
       * \brief Default constructor.
       */
      PreviousStep() : timestamp(0) {};

      /**
       * \brief Container for the previous robot feedback values.
       */
      proto::Feedback feedback;

      /**
       * \brief Container for the previous robot planned values.
       */
      proto::Planned planned;

      /**
       * \brief Previous robot message's timestamp.
       */
      unsigned int timestamp;
    };

    /**
     * \brief Estimate the sample time.
     *
     * \return double with the estimated sample time.
     */
    double estimateSampleTime()
    {
      return (current.timestamp - previous.timestamp)*egm_common_values::conversions::MS_TO_S;
    }

    /**
     * \brief Check the current and the target quaternions for the shortest way (for the interpolation, if used).
     */
    void checkQuaternions();

    /**
     * \brief Check if the robot is close enough to the current point.
     *
     * \return bool indicating if the current target has been reached.
     */
    bool conditionMet(EGMInterfaceConfiguration::InterfaceModes mode);

    /**
     * \brief Initialize the data for a new communication session.
     *
     * \param p_egm_messages containing the recieved message.
     */
    void initializeData(EGMMessageManager* p_egm_messages);

    /**
     * \brief Update the data for a continued communication session.
     *
     * \param p_egm_messages containing the recieved message.
     */
    void updateData(EGMMessageManager* p_egm_messages);

    /**
     * \brief Update the previous step with the current step.
     */
    void updatePrevious();

    /**
     * \brief Validate a new target retrived from the current trajectory. Also fills out missing data.
     */
    void validateTarget();

    /**
     * \brief Reset the current references.
     */
    void resetReferences();

    /**
     * \brief Current motion step. 
     */
    CurrentStep current;

    /**
     * \brief Current goal. 
     */
    Goal goal;

    /**
     * \brief Previous motion step. 
     */
    PreviousStep previous;

    /**
     * \brief The robot controller's status.
     */
    proto::RobotStatus robot_status;
  };

  /**
   * \brief Struct for containing pending events.
   */
  struct PendingMotionEvents
  {
    /**
     * \brief Default constructor.
     */
    PendingMotionEvents();

    /**
     * \brief A flag indicating if the current trajectories should be discarded.
     */
    boost::atomic<bool> do_discard;

    /**
     * \brief A flag indicating if the an evasive motion should be performed.
     */
    boost::atomic<bool> do_evasion;

    /**
     * \brief A flag indicating if the current valid trajectories should be overwritten.
     */
    boost::atomic<bool> do_override;

    /**
     * \brief A flag indicating if the current motion should be stopped.
     *
     * I.e. the current speed should be ramped down to zero.
     */
    boost::atomic<bool> do_stop;
  };

  /**
   * \brief Struct for containing the overall planned motions.
   */
  struct PlannedMotion
  {
    /**
     * \brief Struct for containing trajectories. 
     */
    struct Trajectories
    {
      /**
       * \brief A mutex for protecting the interface's trajectories.
       */
      boost::mutex mutex;

      /**
       * \brief The current trajectory being executed.
       */
      boost::shared_ptr<EGMTrajectory> p_current;

      /**
       * \brief The trajectory to use after an evasive motion.
       */
      boost::shared_ptr<EGMTrajectory> p_evasive;
    };

    /**
     * \brief Struct for containing queues. 
     */
    struct Queues
    {
      /**
       * \brief Default constructor.
       */
      Queues(): primary_queue_size(0) {};

      /**
       * \brief Static constant for the trajectory queues' max number of trajectories.
       */
      static const int MAX_NUMBER_OF_TRAJECTORIES = 200;

      /**
       * \brief Number of trajectories in the primary queue.
       */
      boost::atomic<size_t> primary_queue_size;

      /**
       * \brief A queue of trajectories to execute.
       */
      boost::lockfree::spsc_queue<boost::shared_ptr<EGMTrajectory>,
                                  boost::lockfree::capacity<MAX_NUMBER_OF_TRAJECTORIES> > primary;

      /**
       * \brief A queue of temporary trajectories to execute (while a "discard trajectories" has been ordered).
       */
      boost::lockfree::spsc_queue<boost::shared_ptr<EGMTrajectory>,
                                  boost::lockfree::capacity<MAX_NUMBER_OF_TRAJECTORIES> > temporary;
    };

    /**
     * \brief Checks if there is an currently executing trajectory.
     *
     * \return bool indicating if there is an currently executing trajectory.
     */
    bool hasCurrentTrajectory() const
    {
      return trajectories.p_current != NULL;
    }

    /**
     * \brief Pops one trajectory from the primary queue and stores it as the currently executing trajectory. 
     */
    void popPrimaryQueue()
    {
      queues.primary.pop(trajectories.p_current);
      --queues.primary_queue_size;
    }

    /**
     * \brief Transfer all trajecotries in the temporary queue to the primary queue.
     */
    void transferTemporaryToPrimaryQueue()
    {
      boost::shared_ptr<EGMTrajectory> dummy;
      while (queues.temporary.pop(dummy))
      {
        queues.primary.push(dummy);
        ++queues.primary_queue_size;
      }
    }

    /**
     * \brief Empties the primary queue.
     */
    void emptyPrimaryQueue()
    {
      boost::shared_ptr<EGMTrajectory> dummy;
      while (queues.primary.pop(dummy))
      {
        --queues.primary_queue_size;
      }
    }

    /**
    * \brief Empties the temporary queue.
    */
    void emptyTemporaryQueue()
    {
      boost::shared_ptr<EGMTrajectory> dummy;
      while (queues.temporary.pop(dummy)){}
    }

    /**
     * \brief Checks if the primary queue is empty.
     *
     * \return bool indicating if the primary queue is empty or not.
     */
    bool primaryQueueEmpty()
    {
      return queues.primary_queue_size == 0;
    }

    /**
     * \brief Retrives a point from the currently active trajectory.
     *
     * \param p_point for storing the retrived point.
     *
     * \return bool indicating if a point was retrived.
     */
    bool retriveNextTrajectoryPoint(proto::TrajectoryPoint* p_point);

    /**
     * \brief The currently active trajectories.
     */
    Trajectories trajectories;

    /**
     * \brief The trajectory queues.
     */
    Queues queues;
  };

  /**
   * \brief Function for handling callback requests from an EGM server.
   *
   * \Param server_data contains EGM server data.
   *
   * \return a string containing the reply.
   */
  std::string callbackFunction(const EGMServerData server_data);

  /**
   * \brief Setup data when a new message has been recieved from the robot controller.
   */
  void setupData();

  /**
   * \brief Update the current target.
   */
  void updateCurrentTarget();

  /**
   * \brief Retrive the next point from the current trajectory.
   */
  void retriveNextTrajectoryPoint();

  /**
   * \brief Process the current target point (e.g. use interpolation).
   */
  void processCurrentTarget();

  /**
   * \brief Creates demo references.
   */
  void demoCreateEGMReferences();

  /**
   * \brief Creates demo quaternion references.
   *
   * \param t for the interpolation parameter. 0 <= t <= 1.
   */
  void demoInterpolateQuaternions(double t);

  /**
   * \brief The interface's current state.
   */
  InterfaceState current_state_;

  /**
   * \brief The interface's auxiliary data.
   */
  Auxiliary auxiliary_;

  /**
   * \brief The interface's configuration.
   */
  Configuration configuration_;

  /**
  * \brief The interface's direct references.
  */
  DirectReferences direct_references_;

  /**
   * \brief The interface's motion step.
   */
  MotionStep motion_step_;

  /**
   * \brief The interface's pending events.
   */
  PendingMotionEvents pending_events_;

  /**
   * \brief The interface's planned motion.
   */
  PlannedMotion planned_motion_;

  /**
   * \brief A server for managing the communication with the robot controller.
   */
  EGMServer egm_server_;

  /**
   * \brief Container for the robot's inbound and outbound messages.
   */
  EGMMessageManager egm_messages_;

  /**
   * \brief A object to manage the interpolation (used for simple interpolation and when ramping out the speed).
   */
  EGMSimpleInterpolationManager simple_interpolator_;

  /**
   * \brief Stream object for logging data.
   */
  std::ofstream log_stream_;
};

} // end namespace egm_interface
} // end namespace abb

#endif // EGM_INTERFACE_DEFAULT_H
