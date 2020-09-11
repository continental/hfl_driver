// Copyright 2020 Continental AG
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
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
//  * Neither the name of Continental AG nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

///
/// @file hfl110dcu-test.cpp
///
/// @brief This file defines the HFL110DCU ROS unit tests
///
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "camera_commander/camera_commander.h"
#include "ros/ros.h"
///
/// ################################
///           Declare Tests
/// ################################
///

TEST(HFL110DCUTestSuite, testTEST)
{
  ASSERT_EQ("true", "true");
}

TEST(HFL110DCUTestSuite, testDefaultParams)
{
  // TODO(evan_flynn): this assumes the launch file defaults are set to HFL110DCU and left
  // for model and frame_id
  ros::NodeHandle nh("hfl110dcu_01");
  std::string model, version, frame_id, parent_frame, interface;
  std::string camera_ip, computer_ip;
  int frame_port, object_port;

  nh.getParam("model", model);
  nh.getParam("version", version);
  nh.getParam("frame_id", frame_id);
  nh.getParam("parent_frame", parent_frame);
  nh.getParam("ethernet_interface", interface);
  nh.getParam("camera_ip_address", camera_ip);
  nh.getParam("computer_ip_address", computer_ip);
  nh.getParam("frame_data_port", frame_port);
  nh.getParam("object_data_port", object_port);

  ASSERT_EQ(model, "hfl110dcu");
  ASSERT_EQ(version, "v1");
  ASSERT_EQ(frame_id, "hfl110dcu_01");
  ASSERT_EQ(interface, "eno1");
  ASSERT_EQ(camera_ip, "192.168.10.21");
  ASSERT_EQ(computer_ip, "192.168.10.5");
  ASSERT_EQ(frame_port, 57410);
  ASSERT_EQ(object_port, 57411);
}

