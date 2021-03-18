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
/// @file base_hfl110dcu.cpp
///
/// @brief This file defines HFL110DCU cameras base class.
///
#include <base_hfl110dcu.h>

#include <string>

namespace hfl
{
bool BaseHFL110DCU::getConfiguration(std::string model, std::string version)
{
  std::string full_model = model + version;
  // Return false if camera registers not found
  if (REGS_OFFSET_ADDRS.find(model) == REGS_OFFSET_ADDRS.end())
  {
    std::cout << "[ERROR]" << model << " not available" << std::endl;
    return false;
  }
  // Return false if model not found
  if (CAMERA_MODELS.find(full_model) == CAMERA_MODELS.end())
  {
    std::cout << "[ERROR]"
              << " configuration registers for " << version << " not available" << std::endl;
    return false;
  }
  // Return false if mode register not found
  if (MODE_REGISTERS.find(full_model) == MODE_REGISTERS.end())
  {
    std::cout << "[ERROR]" << full_model << " mode register not available" << std::endl;
    return false;
  }
  // Set current model and version
  model_ = model;
  version_ = version;
  // Set frame configurations
  frame_.reset(new Frame(FRAME_ROWS, FRAME_COLUMNS, PIXEL_RETURNS, PIXEL_SLICES));
  frame_->intensity_bits_ = INTENSITY_BITS;
  frame_->range_bits_ = RANGE_BITS;
  frame_->id_ = FRAME_ID;
  // frame_ changed to frame_ as referenced in the hfl_interface.h class
  return true;
}

bool BaseHFL110DCU::setGlobalRangeOffset(double offset)
{
  try {
    global_offset_ = offset * 256;
    return true;
  } catch (const std::exception& e) {
    return false;
  }
}
}  // namespace hfl
