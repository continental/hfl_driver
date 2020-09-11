// Copyright 2020 Continental AG
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


///
/// @file camera_commander.h
///
/// @brief This file defines the CameraCommander class
///
#ifndef CAMERA_COMMANDER__CAMERA_COMMANDER_H_
#define CAMERA_COMMANDER__CAMERA_COMMANDER_H_

#include <hfl_driver/HFLConfig.h>
#include <hfl_interface.h>

#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>

#include <vector>
#include <string>
#include <memory>

#include "udp_com/UdpPacket.h"
#include "udp_com/UdpSend.h"
#include "udp_com/UdpSocket.h"

#include "ros/ros.h"

namespace hfl
{
/// Commander states enum
enum commander_states
{
  state_probe = 0,
  state_init,
  state_done,
  state_error
};

/// Error Codes
enum error_codes
{
  no_error = 0,
  frame_socket_error,
  object_socket_error,
  lut_socket_error
};

///
/// @brief Implements the camera configuration and setup
///
class CameraCommander : public nodelet::Nodelet
{
public:
  ///
  /// CameraCommander constructor
  ///
  CameraCommander();

  ///
  /// CameraCommander destructor
  ///
  ~CameraCommander();

  ///
  /// Initialize Nodelet member variables
  ///
  /// @return void
  ///
  void onInit();

  ///
  /// Initialize UDP topics and services
  ///
  /// @return void
  ///
  bool udpInit();

  ///
  /// Create Socket Request Function
  ///
  /// @param[in] computerAddr Computer's IP Address
  /// @param[in] cameraAddr Camera's IP Address
  /// @param[in] port UDP port number
  /// @param[in] isMulticast describes if the connection is multicast
  /// @return bool true if socket created
  ///
  bool createSocket(std::string computerAddr, std::string cameraAddr,
      uint16_t port, bool isMulticast);

private:
  /// Node Handle
  ros::NodeHandle node_handler_;

  /// Namespace
  std::string namespace_;

  /// UDP Frame Data subscriber
  ros::Subscriber frame_data_subscriber_;

  /// UDP Object Data Subscriber
  ros::Subscriber object_data_subscriber_;

  /// UDP LUT Data Subscriber
  ros::Subscriber lut_data_subscriber_;

  /// UDP send service client
  ros::ServiceClient udp_send_service_client_;

  /// UDP socket service client
  ros::ServiceClient udp_socket_creation_service_client_;

  /// Dynamic Reconfigure server
  std::shared_ptr<dynamic_reconfigure::Server<hfl_driver::HFLConfig> > dynamic_parameters_server_;

  /// Status checker timer
  ros::Timer timer_;

  /// Commander current state
  commander_states current_state_;

  /// Commander Previous state prior to error
  commander_states previous_state_;

  /// Error Status
  error_codes error_status_;

  /// Ethernet Interface
  std::string ethernet_interface_;

  /// IP Address of sensor
  std::string camera_address_;

  /// IP Address of computer
  std::string computer_address_;

  /// Frame Data UDP port
  int frame_data_port_;

  /// Object Data UDP port
  int object_data_port_;

  /// Pointer to Flash camera
  std::shared_ptr<hfl::HflInterface> flash_;

  ///
  /// Sets CameraCommander current state
  ///
  /// @param[in] timer_event ROS TimerEvent object
  ///
  /// @return void
  ///
  void setCommanderState(const ros::TimerEvent& timer_event);

  ///
  /// Loads the HFL camera instace accordingly
  /// to the launch parameters
  ///
  /// @return bool true if flash loading succed
  ///
  bool setFlash();

  ///
  /// Callback for UDP frame data packets.
  ///
  /// Receives the frame data messages for parsing
  /// through IP addresses validation.
  ///
  /// @param[in] udp_packet UDP packet message
  ///
  /// @return void
  ///
  void frameDataCallback(const udp_com::UdpPacket& udp_packet);

  ///
  /// Callback for UDP object data packets
  ///
  /// Receives the object data messages for parsing
  /// through IP addresses validation
  ///
  /// @param[in] udp_packet UDP packet message
  ///
  /// @return void
  ///
  void objectDataCallback(const udp_com::UdpPacket& udp_packet);

  ///
  /// Callback for UDP LUT data packets
  ///
  /// Receives the lut data messages for parsing
  /// through IP addresses validation
  ///
  /// @param[in] udp_packet UDP packet message
  ///
  /// @return void
  ///
  void lutDataCallback(const udp_com::UdpPacket& udp_packet);

  ///
  /// Uses the udp_com service binded send function for
  /// command sending
  ///
  /// @param[in] data Data array to be sent
  ///
  /// @return bool true if command sent succed
  ///
  bool sendCommand(const std::vector<uint8_t>& data);

  ///
  /// Uses the dynamic reconfigure service binded callback
  /// dynamic parameter configuration.
  ///
  /// @param[in] config parameters data
  /// @param[in] level priority
  ///
  /// @return void
  ///
  void dynamicPametersCallback(hfl_driver::HFLConfig& config, uint32_t level);

  ///
  /// Checks for Errors and returns the designated error code
  ///
  error_codes checkForError();

  ///
  /// Fixes the error corresponding to the error code
  /// Returns true if error fixed, false otherwise
  /// @return bool
  ///

  bool fixError(error_codes error);
};

}  // namespace hfl

#endif  // CAMERA_COMMANDER__CAMERA_COMMANDER_H_
