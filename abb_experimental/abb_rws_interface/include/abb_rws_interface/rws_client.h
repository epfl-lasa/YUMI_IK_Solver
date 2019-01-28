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

#ifndef RWS_CLIENT_H
#define RWS_CLIENT_H

#include <vector>

#include "rws_curl_client.h"

namespace abb
{
namespace rws_interface
{
/**
 * \brief A class for a Robot Web Services (RWS) client based on a cURL HTTP client.
 *
 * TODO: Look into using XML or JSON parser instead.
 */
class RWSClient : public CURLClient
{
public:
  /**
   * \brief A constructor.
   *
   * \param ip_address for the remote HTTP server's IP address.
   * \param port for the remote HTTP server's port.
   */
  RWSClient(const std::string ip_address, const std::string port = "80")
  :
  CURLClient("Default User", "robotics", ip_address, port)
  {
  }

  /**
   * \brief A method for retriving the value of a RAPID IO signal.
   * 
   * \param iosignal for the IO signal's name.
   *
   * \return std::string containing the response.
   */
  std::string getIOSignal(const std::string iosignal);

  /**
   * \brief A method for retriving the data of a RAPID symbol.
   * 
   * \param rapid_task for the name of the RAPID task containging the symbol.
   * \param rapid_module for the name of the RAPID module containging the symbol.
   * \param symbol_name for symbol's name.
   *
   * \return std::string containing the response.
   */
  std::string getRAPIDSymbolData(const std::string rapid_task,
                                 const std::string rapid_module,
                                 const std::string symbol_name);

  /**
   * \brief A method for retriving the execution state of RAPID.
   * 
   * \return std::string containing the response.
   */
  std::string getRAPIDExecution();

  /**
   * \brief A method for retriving the operation mode of the controller.
   * 
   * \return std::string containing the response.
   */
  std::string getPanelOperationMode();

  /**
   * \brief A method for setting the value of a RAPID IO signal.
   * 
   * \param iosignal for the IO signal's name.
   * \param value for the IO signal's new value.
   */
  void setIOSignal(const std::string iosignal, const std::string value);

  /**
   * \brief A method for setting the data of a RAPID symbol.
   * 
   * \param rapid_task for the name of the RAPID task containging the symbol.
   * \param rapid_module for the name of the RAPID module containging the symbol.
   * \param symbol_name for symbol's name.
   * \param value for the RAPID symbol's new value.
   */
  void setRAPIDSymbolData(const std::string rapid_task,
                          const std::string rapid_module,
                          const std::string symbol_name,
                          const std::string data);

  /**
   * \brief A method for extracting substrings from a string.
   * 
   * \param input of the whole string.
   *
   * \return std::vector<std::string> containing the extracted substrings.
   */
  std::vector<std::string> extractDelimitedSubstrings(const std::string input);

  /**
   * \brief A method for extracting values from a delimited string.
   * 
   * \param input of the whole string.
   *
   * \return std::vector<std::string> containing the extracted values.
   */
  std::vector<std::string> extractDelimitedValues(const std::string input);
};

} // end namespace rws_interface
} // end namespace abb

#endif