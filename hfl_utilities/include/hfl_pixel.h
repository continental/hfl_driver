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
/// @file hfl_pixel.h
///
/// @brief This file defines the frame's classes components.
///

#ifndef HFL_PIXEL_H_
#define HFL_PIXEL_H_

#include <iostream>
#include <vector>

namespace hfl
{
/// Pixel's Slice data type
using Slice = std::vector<uint16_t>;

///
/// @brief Data structure for pixel returns.
///
struct PixelReturn
{
  /// Return's range value
  float range{ 0.0 };

  /// Return's intensity value
  uint16_t intensity{ 0 };
};

///
/// @brief Storages and handles the pixel data component.
///
class Pixel
{
public:
  ///
  /// Pixel initializer constructor
  ///
  /// @param returns_size Number of returns per pixel
  /// @param slices_size Number of slices per pixel
  ///
  Pixel(size_t returns_size, size_t slices_size);

  ///
  /// Return the Pixel's return data at location x
  ///
  /// @param x Desired pixel return
  /// @return PixelReturn data structure
  ///
  PixelReturn& atReturn(uint16_t x);

  ///
  /// Returns the Slice data value at location x
  ///
  /// @param x Desired Slice position
  /// @return Slice data value
  ///
  uint16_t& atSlice(uint16_t x);

private:
  /// Slices data
  Slice slices_;

  /// Pixel's returns data
  std::vector<PixelReturn> returns_;
};

}  // namespace hfl
#endif  // HFL_PIXEL_H_
