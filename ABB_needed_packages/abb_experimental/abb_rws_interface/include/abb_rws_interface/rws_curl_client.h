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
 
#ifndef RWS_CURL_CLIENT_H
#define RWS_CURL_CLIENT_H

#include <string>

#include <boost/thread.hpp>

#include <curl/curl.h>

namespace abb
{
namespace rws_interface
{
/**
 * \brief A class for a simple HTTP client based on cURL.
 */
class CURLClient
{
public:
  /**
   * \brief A constructor.
   *
   * \param user for the remote HTTP server's authentication (here assumed to be Digest).
   * \param password for the remote HTTP server's authentication (here assumed to be Digest).
   * \param ip_address for the remote HTTP server's IP address.
   * \param port for the remote HTTP server's port.
   */
  CURLClient(const std::string user,
             const std::string password,
             const std::string ip_address,
             const std::string port = "80")
  :
  user_(user),
  password_(password),
  ip_address_(ip_address),
  port_(port),
  base_url_("http://" + ip_address + ":" + port),
  curl_(NULL)
  {
    curl_ = curl_easy_init();
  }

  /**
  * \brief A destructor
  */
  ~CURLClient()
  {
    curl_easy_cleanup(curl_);
  }

  /**
   * \brief A method for sending a HTTP GET message.
   * 
   * \param url_path_and_query for the GET message's remaining URL part.
   * \param result containing the HTTP server's reply.
   *
   * \return CURLcode containing the CURLcode result from a curl_easy_perform(...) call.
   */
  CURLcode get(const std::string url_path_and_query, std::string* result);

  /**
   * \brief A method for sending a HTTP POST message.
   * 
   * \param url_path_and_query for the POST message's remaining URL part.
   * \param post_data containing the data to POST.
   *
   * \return CURLcode containing the CURLcode result from a curl_easy_perform(...) call.
   */
  CURLcode post(const std::string url_path_and_query, const std::string post_data = "");

  /**
   * \brief A method for retriving a substring in a string.
   * 
   * \param whole_string for the string containing the substring.
   * \param substring_start start of the substring.
   * \param substring_end end of the substring.
   *
   * \return string containing the substring.
   */
  std::string findSubstringContent(const std::string whole_string,
                                   const std::string substring_start,
                                   const std::string substring_end);

private:
  /**
   * \brief An enum for specifying a GET or POST message.
   */
  enum CURLCase
  {
    GET, ///< GET message.
    POST ///< POST message.
  };

  /**
   * \brief A callback method for retriving header data.
   * 
   * \param buffer for the recieved HTTP message's container.
   * \param size of a item to add to the userdata.
   * \param nitems number of items to add to the userdata.
   * \param userdata for recieving the resulting data.
   *
   * \return size_t for the size of all items added to the userdata.
   */
  static size_t headerCallback(char* buffer, size_t size, size_t nitems, void* userdata);

  /**
   * \brief A callback method for retriving body data.
   * 
   * \param ptr for the recieved HTTP message's container.
   * \param size of a item to add to the userdata.
   * \param nmemb number of items to add to the userdata.
   * \param userdata for recieving the resulting data.
   *
   * \return size_t for the size of all items added to the userdata.
   */
  static size_t writeCallback(char* ptr, size_t size, size_t nmemb, void* userdata);

  /**
   * \brief A method for setting up the cURL options.
   * 
   * \param curl for the cURL object.
   * \param curl_case specifying GET or POST messsage.
   * \param url for the URL to use in the HTTP message.
   * \param post_data for the data used in a POST message.
   */
  void setupCURLOptions(CURL* curl, const CURLCase curl_case, const std::string url, const char* post_data = "");

  /**
   * \brief Method to clear the callback buffers.
   */
  void clearBuffers();

  /**
   * \brief A mutex for protecting the object's resources.
   */
  boost::mutex mutex_;

  /**
   * \brief The Digest authentication user.
   */
  std::string user_;

  /**
   * \brief The Digest authentication password.
   */
  std::string password_;

  /**
   * \brief The HTTP server's IP address.
   */
  std::string ip_address_;

  /**
   * \brief The HTTP server's IP port.
   */
  std::string port_;

  /**
   * \brief The base URL for the HTTP messages.
   */
  std::string base_url_;

  /**
   * \brief Buffer for the HTTP messages' header data.
   */
  std::string headerdata_buffer_;

  /**
   * \brief Buffer for the HTTP messages' body data.
   */
  std::string writedata_buffer_;

  /**
  * \brief Pointer to a CURL object.
  */
  CURL* curl_;
};

} // end namespace rws_interface
} // end namespace abb

#endif