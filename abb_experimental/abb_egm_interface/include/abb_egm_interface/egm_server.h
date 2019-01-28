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

#ifndef EGM_SERVER_H
#define EGM_SERVER_H

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "egm_common.h"

namespace abb
{
namespace egm_interface
{
/**
 * \brief An abstract class for an EGM interface.
 */
class AbstractEGMInterface
{
/**
 * \brief A friend to the interface.
 */
friend class EGMServer;

private:
  /**
   * \brief Pure virtual function for handling callback requests from an EGM server.
   *
   * \Param data contains EGM server data.
   *
   * \return a string containing the reply.
   */
  virtual std::string callbackFunction(const EGMServerData data) = 0;
};




/**
 * \brief A class for an Externally Guided Motion (EGM) UDP server.
 *
 * A class for an asynchronous EGM UDP server.
 * The server recieves EGM messages from an ABB robot controller using EGM
 * and reply with EGM messages containing the new references for the robot.
 */
class EGMServer
{
public:
  /**
   * \brief A constructor.
   *
   * \param io_service for operating boost asio's asynchronous functions.
   * \param port_number for the server's UDP socket.
   * \param egm_interface that processes the recieved messages.
   */
  EGMServer(boost::asio::io_service& io_service,
            size_t port_number,
            AbstractEGMInterface* p_egm_interface);

  /**
   * \brief A destructor.
   */
  ~EGMServer();

private:
  /**
   * \brief Method for starting an asynchronous recieve.
   */
  void startAsynchronousRecieve();

  /**
   * \brief Callback method for handling an asynchronous recieve.
   *
   * \param error for containing an error code.
   * \param bytes_transferred is the number of bytes recieved.
   */
  void recieveCallback(const boost::system::error_code& error, const std::size_t bytes_transferred);

  /**
   * \brief Callback method for handling an asynchronous send.
   *
   * \param error for containing an error code.
   * \param bytes_transferred is the number of bytes transmitted.
   */
  void sendCallback(const boost::system::error_code& error, const std::size_t bytes_transferred);

  /**
   * \brief Resets the buffer.
   */
  void resetBuffer();

  /**
   * \brief Static constant for the socket's buffer size.
   */
  static const size_t BUFFER_SIZE = 1024;

  /**
   * \brief The server's UDP socket.
   */
  boost::shared_ptr<boost::asio::ip::udp::socket> p_socket_;

  /**
   * \brief The address of the calling computer (e.g. the robot controller).
   */
  boost::asio::ip::udp::endpoint remote_endpoint_;

  /**
   * \brief An array buffer for storing the server's serialized inbound messages (i.e. the robot's outbound messages).
   */
  boost::array<char, BUFFER_SIZE> recieve_buffer_;
  
  /**
   * \brief A pointer to an object that is derived from AbstractEGMInterface. It processes the recieved messages.
   */
  AbstractEGMInterface* p_egm_interface_;

  /**
   * \brief Container for server data.
   */
  EGMServerData server_data_;
};

} // end namespace egm_interface
} // end namespace abb

#endif // EGM_SERVER_H
