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
/// @file base_hfl110dcu.h
///
/// @brief This file defines the HFL110DCU camera base class.
///
#ifndef BASE_HFL110DCU_H_
#define BASE_HFL110DCU_H_
#include <hfl_interface.h>
#include <string>
#include <vector>

namespace hfl
{
/// Default frame rows
const uint16_t FRAME_ROWS{ 32 };
/// Default frame cols
const uint16_t FRAME_COLUMNS{ 128 };
/// Default frame cols
const uint16_t PIXEL_RETURNS{ 1 };
/// Default frame cols
const uint16_t PIXEL_SLICES{ 128 };
/// Default words per UDP packet
const uint32_t WORDS_PER_PACKET{ 0x168 };
///  Default bits used for intensity
const uint8_t INTENSITY_BITS{ 5 };
/// Default bits used for range
const uint8_t RANGE_BITS{ 8 };
/// Default bits used for range presicion
const uint8_t RANGE_PRECISION_BITS{ 6 };
/// Default bits used for intensity publishing
const uint8_t INTENSITY_PUBLISH_BITS{ 12 };
/// Default frame ID
const char FRAME_ID[] = "hfl110dcu";
/// Default camera intrinsics
const char CAMERA_INTRINSICS[] = "min000000";
/// Default expected memory address
const uint32_t EXPECTED_ADDRESS{ 0xffffffff };

///
/// @brief Base class for the HFL110DCU cameras
///
class BaseHFL110DCU : public HflInterface
{
public:
  ///
  /// Sets the specified frame rate.
  ///
  /// @param[in] rate Frame rate to be set
  ///
  /// @return bool true if given frame rate set
  ///
  bool setFrameRate(double rate) override
  {
    return false;
  }

  ///
  /// Returns the current frame rate.
  ///
  /// @return current frame rate
  ///
  double getFrameRate(bool reg_format = false) const
  {
    return 25.0;
  };

  ///
  /// Sets global range offset
  ///
  /// @param[in] offset global range offset to set
  ///
  /// @return bool true if given global range offset is set
  ///
  bool setGlobalRangeOffset(double offset);

  ///
  /// Sets channel range offset
  ///
  /// @param[in] ch channel number
  /// @param[in] offset channel range offset to set
  ///
  /// @return bool true if given channel range offset is set
  ///
  bool setChannelRangeOffset(uint8_t ch, double offset);

  ///
  /// Sets intensity range offset
  ///
  /// @param[in] band intensity range number
  /// @param[in] offset channel range offset to set
  ///
  /// @return bool true if given intensity band range offset is set
  ///
  virtual bool setIntensityRangeOffset(uint8_t band, double offset);

protected:
  /// Range Magic Number
  double range_magic_number_;

  /// Current mode parameters
  Attribs_map mode_parameters;

  /// UDP sender function
  std::function<void(const std::vector<uint8_t>&)> udp_send_function_;

  /// HFL110DCU camera memory_types
  enum HFL110DCU_memory_types
  {
    mem_ri = 0,
    types_size
  };

  ///
  /// Gets the available memory modes, its params and register
  /// offset addresses.
  ///
  /// @param model The model of the current lidar
  /// @param version The HFL110DCU SW version
  ///
  /// @return Available memory modes
  ///
  bool getConfiguration(std::string model, std::string version);

  ///
  /// Parse packet into objects
  ///
  /// @param[in] start_byte starting byte, packet packet data to parse
  ///
  /// @return bool true if successfully parsed object data
  ///
  virtual bool parseObjects(int start_byte, const std::vector<uint8_t>& packet) = 0;

  ///
  /// Process the object data from udp packets
  ///
  /// @param[in] data object data
  ///
  /// @return bool
  ///
  virtual bool processObjectData(const std::vector<uint8_t>& data) = 0;
};
}  // namespace hfl

#endif  // BASE_HFL110DCU_H_
