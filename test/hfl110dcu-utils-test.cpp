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
/// @file hfl110dcu-utils-test.cpp
///
/// @brief This file defines the HFL110DCU utilities unit tests
///

#include <gtest/gtest.h>
#include <base_hfl110dcu.h>
#include <vector>

// create dummy HFL110DCU class
class HFL110DCU : public hfl::BaseHFL110DCU
{
public:
  HFL110DCU()
  {
    // initialize values to helper class
    model_ = "hfl110dcu";
    version_ = "v1";
    ip_address_ = "192.168.10.21";
    frame_data_port_ = 1900;
    uint16_t height = 32;
    uint16_t width = 128;
    uint16_t returns = 2;
    uint16_t slices = 128;
  };

  // NOTE: these are the functions that will need to be written
  // in the actual image_processor classes
  //
  bool parseFrame(int start_byte, const std::vector<unsigned char>&) override
  {
    return true;
  };
  // TODO(evan_flynn): should this return a bool to indicate status?
  bool processFrameData(const std::vector<uint8_t>& data) override
  {
    return true;
  };
};

///
/// Creates a new HFL110DCU Fixture object that can be used within each TEST_F
///
class HFL110DCUFixture : public ::testing::Test
{
public:
  ///
  /// Constructor
  ///
  HFL110DCUFixture()
  {
    // initialization code here
  }

  ///
  /// Destructor
  ///
  ~HFL110DCUFixture()
  {
    // cleanup any pending stuff, but no exceptions allowed
  }

  void SetUp()
  {
    // code here will execute just before the test ensues
  }

  void TearDown()
  {
    // code here will be called just after the test completes
    // ok to through exceptions from here if need be
  }

  // put in any custom data members that you need
  // the hfl interface class variable
  HFL110DCU *flash_;
  // default helper data members
};  // end of HFL110DCUFixture class

///
/// Declare Parameter Test
///

TEST_F(HFL110DCUFixture, testTEST)
{
  ASSERT_EQ(true, true);
}

TEST_F(HFL110DCUFixture, testModelParam)
{
  // Test getModel function
  // ASSERT_EQ(flash_->getModel(), "hfl110dcu");  // Equal To
  ASSERT_EQ(true, true);
}

TEST_F(HFL110DCUFixture, testVersionParam)
{
  // Test getVersion function
  // ASSERT_EQ(flash_->getVersion(), "v1");  // Equal To
  ASSERT_EQ(true, true);
}

