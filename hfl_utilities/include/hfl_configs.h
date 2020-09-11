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
/// @file hfl_configs.h
///
/// @brief This file defines HFL cameras data and custom types.
///
#ifndef HFL_CONFIGS_H_
#define HFL_CONFIGS_H_

#include <iostream>
#include <string>
#include <functional>
#include <vector>
#include <map>
#include <memory>
#include <utility>

namespace hfl
{
///  Mode parameters map
using Attribs_map = std::map<std::string, float>;

/// Camera modes map
using Configs_map = std::map<std::string, Attribs_map>;

/// HFL cameras map
using Setups_map = std::map<std::string, Configs_map>;

/// Register bit's values vector
using Regs_bits_vec = std::vector<std::pair<std::string, int>>;

/// Cameras registers map
using Registers_map = std::map<std::string, Regs_bits_vec>;

/// HFL cameras register addresses
const Configs_map REGS_OFFSET_ADDRS
{
  { "hfl110dcu", {} }
};

/// HFL cameras memory maps and parameters
const Setups_map CAMERA_MODELS =
{
  { "hfl110dcuv1",
    {
      { "RI",
        {
          { "start_address", 0x00000000 }
        }
      }
    }
  }
  // Insert new camera models here
};

/// HFL cameras mode registers
const Registers_map MODE_REGISTERS =
{
  { "hfl110dcuv1",
    {
        // BLANK
    }
  }
};

}  // namespace hfl

#endif  //  HFL_CONFIGS_H_
