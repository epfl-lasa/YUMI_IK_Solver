/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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

#include <signal.h>

#include <curl/curl.h>

#include "abb_egm_driver/abb_joint_trajectory_actions.h"

using namespace abb::joint_trajectory_action;

boost::shared_ptr<JointTrajectoryActionAbstract> p_action;
boost::asio::io_service io_service;

void cleanup(int sig)
{
  curl_global_cleanup();

  io_service.stop();
  while(!io_service.stopped()) {}

  p_action.reset();

  ros::shutdown();
}

int main(int argc, char** argv)
{  
  // Initialize node.
  ros::init(argc, argv, "joint_trajectory_action", ros::init_options::NoSigintHandler);

  // Register a custom signal to clean up on INTERRUPT signal.
  signal(SIGINT, cleanup);

  std::string robot_ip_address = "";
  ros::param::get(ros::this_node::getName() + "/robot_ip_address", robot_ip_address);

  std::string rws_port = "";
  ros::param::get(ros::this_node::getName() + "/robot_web_services_port", rws_port);


  bool dual_armed = false;
  ros::param::get(ros::this_node::getName() + "/dual_armed", dual_armed);

  CURLcode curl_code = curl_global_init(CURL_GLOBAL_DEFAULT);

  if(curl_code == CURLE_OK)
  {
    boost::thread_group worker_threads;

    if(dual_armed)
    {
      p_action.reset(new JointTrajectoryActionDualArmed(io_service, robot_ip_address, rws_port));
    }
    else
    {
      p_action.reset(new JointTrajectoryActionSingleArm(io_service, robot_ip_address, rws_port));
    }

    for(size_t i = 0; i < JointTrajectoryActionAbstract::MAX_NUMBER_OF_EGM_CONNECTIONS_; ++i)
    {
      worker_threads.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
    }

    p_action->run();

    worker_threads.join_all();
  }
  else
  {
    ROS_ERROR_STREAM("[" << ros::this_node::getName() << "] - Shutting down (could not initialize cURL)");
  }

  curl_global_cleanup();

  return 0;
}
