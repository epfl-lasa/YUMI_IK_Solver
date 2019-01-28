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

#include <boost/bind.hpp>

#include "abb_egm_interface/egm_server.h"

namespace abb
{
namespace egm_interface
{
/****************************************************************************************
 * Class definitions: EGMServer
 */

EGMServer::EGMServer(boost::asio::io_service& io_service,
                     size_t port_number,
                     AbstractEGMInterface* p_egm_interface)
:
p_egm_interface_(p_egm_interface)
{
  bool ok = true;
  try
  {
    server_data_.port_number = port_number;
    p_socket_.reset(new boost::asio::ip::udp::socket(io_service,
                                                     boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                                                                    (unsigned short) port_number)));
  }
  catch (std::exception e)
  {
    ok = false;
  }

  if (ok)
  {
    startAsynchronousRecieve();
  }
}

EGMServer::~EGMServer()
{
  p_socket_->close();
  p_socket_.reset();
}

void EGMServer::startAsynchronousRecieve()
{
  resetBuffer();
  p_socket_->async_receive_from(boost::asio::buffer(recieve_buffer_),
                                remote_endpoint_,
                                boost::bind(&EGMServer::recieveCallback,
                                            this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
}

void EGMServer::recieveCallback(const boost::system::error_code& error, const std::size_t bytes_transferred)
{
  server_data_.message = std::string(recieve_buffer_.begin(), recieve_buffer_.end());
  server_data_.bytes_transferred = bytes_transferred;

  if (error == boost::system::errc::success)
  {
    // Process the recieved data via the callback object (creates the reply message).
    std::string reply = p_egm_interface_->callbackFunction(server_data_);

    // Send the response message to the robot controller.
    p_socket_->async_send_to(boost::asio::buffer(reply),
                             remote_endpoint_,
                             boost::bind(&EGMServer::sendCallback,
                                         this,
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
  }

  // Add another asynchrous operation to the boost io_service object.
  startAsynchronousRecieve();
}

void EGMServer::sendCallback(const boost::system::error_code& error, const std::size_t bytes_transferred) {}

void EGMServer::resetBuffer()
{
  boost::array<char, BUFFER_SIZE>::iterator i;
  for (i = recieve_buffer_.begin(); i != recieve_buffer_.end(); ++i)
  {
    *i = 0;
  }
}

} // end namespace egm_interface
} // end namespace abb
