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
/// @file hfl110dcu.h
///
/// @brief This file defines the HFL110DCU image processor class.
///
#ifndef IMAGE_PROCESSOR__HFL110DCU_H_
#define IMAGE_PROCESSOR__HFL110DCU_H_

#include <base_hfl110dcu.h>

#include <angles/angles.h>
#include <arpa/inet.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <string>
#include <vector>
#include <cmath>
#include <memory>

#include "ros/ros.h"


#define HFL110_MAGIC_NUMBER_16_BIT 0.000762951           // 50 / 2^16

const float NO_RETURN_DISTANCES = NAN;

namespace hfl
{
/// @brief HFL110DCU v1 frame struct
struct PointCloudReturn
{
  uint16_t range;
  uint16_t intensity;
  uint16_t range2;
  uint16_t intensity2;
};

/// @brief HFL110DCU v1 ethernet packet header struct
struct UdpPacketHeader
{
  uint16_t udp_version;
  uint16_t pca_version;
  uint64_t timeStamp;
  uint32_t upd_packet_number;
  uint32_t image_row_number;
};

/// @brief HFL110DCU v1 ethernet extrinsics struct
struct CameraIntrinsics
{
  float_t fx;
  float_t fy;
  float_t ux;
  float_t uy;
  float_t r1;
  float_t r2;
  float_t t1;
  float_t t2;
  float_t r3;
};

/// @brief HFL110DCU v1 ethernet extrinsics struct
struct CameraExtrinsics
{
  float_t intrinsic_yaw;
  float_t intrinsic_pitch;
  float_t extrinsic_yaw;
  float_t extrinsic_pitch;
  float_t extrinsic_roll;
  float_t extrinsic_vertical;
  float_t extrinsic_horizontal;
  float_t extrinsic_distance;
  uint32_t status;
};

/// @brief HFL110DCU v1 ethernet frame struct
struct UdpFrame
{
  UdpPacketHeader header;
  CameraIntrinsics camera_intrinsics;
  CameraExtrinsics camera_extrinsics;
  PointCloudReturn returns[128];
  uint8_t pixel_type[128];
};

/// @brief HFL110DCU v1 object geometry
struct objGeo
{
  float x_rear_r;
  float y_rear_r;
  float x_rear_l;
  float y_rear_l;
  float x_front_l;
  float y_front_l;
  float height;
  float ground_offset;
  float fDistX;
  float fDistY;
  float yaw;
};

/// @brief HFL110DCU v1 object kinematics
struct objKin
{
  float fVabsX;
  float fVabsY;
  float fVrelX;
  float fVrelY;
  float fAabsX;
  float fDistXDistY;
  float fDistXVx;
  float fDistXVy;
  float fDistXAx;
  float fDistXAy;
  float fDistYVx;
  float fDistYVy;
  float fDistYAx;
  float fDistYAy;
  float fVxVy;
  float fVxAx;
  float fVxAy;
  float fVyAx;
  float fVyAy;
  float fAxAy;
};

/// @brief HFL110DCU v1 object state
struct objState
{
  unsigned TP_OBJ_MT_STATE_DELETED : 1;
  unsigned TP_OBJ_MT_STATE_NEW : 1;
  unsigned TP_OBJ_MT_STATE_MEASURED : 1;
  unsigned TP_OBJ_MT_STATE_PREDICTED : 1;
  unsigned TP_OBJ_MT_STATE_INACTIVE : 1;
  unsigned TP_OBJ_MT_STATE_MAX_DIFF : 1;
};

/// @brief HFL110DCU v1 object dynamic property
struct objDyn
{
  unsigned EM_GEN_OBJECT_DYN_PROPERTY_MOVING : 1;
  unsigned EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY : 1;
  unsigned EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING : 1;
  unsigned EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_LEFT : 1;
  unsigned EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_RIGHT : 1;
  unsigned EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN : 1;
  unsigned EM_GEN_OBJECT_DYN_PROPERTY_STOPPED : 1;
  unsigned EM_GEN_OBJECT_DYN_PROPERTY_MAX_DIFF_TYPES : 1;
};

/// @brief HFL110DCU v1 ethernet object struct
struct hflObj
{
  objGeo geometry;
  objKin kinematics;
  objState state;
  objDyn dynamic_props;
  uint8_t quality;
  uint8_t classification;
  uint8_t confidence;
};

/// @brief HFL110DCU v1 telemetry struct
struct telemetry
{
  uint32_t uiHardwareRevision;
  float fSensorTemp;
  float fHeaterTemp;
  uint32_t uiFrameCounter;
  float fADCUbattSW;
  float fADCUbatt;
  float fADCHeaterLens;
  float fADCHeaterLensHigh;
  float fADCTemp0Lens;
  float fAcquisitionPeriod;
  unsigned uiTempSensorFeedback;
  char au8SerialNumber[26];
};

///
/// @brief Implements the HFL110DCU camera image parsing and publishing.
///
class HFL110DCU : public BaseHFL110DCU
{
public:
  ///
  /// HFL110DCU image processor constructor.
  ///
  /// @param[in] model camera hfl model
  /// @param[in] version camera version
  /// @param[in] frame_id camera's coordinate frame name
  /// @param[in] node_handler reference to the ros node handler
  ///
  HFL110DCU(std::string model, std::string version,
            std::string frame_id, ros::NodeHandle& node_handler);

  ///
  /// Parse out the packet data into depth and intensity images
  ///
  /// @param[in] starting byte, packet to parse
  ///
  /// @return bool true if successfully parsed packet
  ///
  bool parseFrame(int start_byte, const std::vector<uint8_t>& packet) override;

  ///
  /// Process data frame from udp packets.
  ///
  /// @param[in] data frame data array
  ///
  /// @return bool true if successful
  ///
  bool processFrameData(const std::vector<uint8_t>& data) override;

  ///
  /// Parse out pdm data from packet
  ///
  /// @param[in] starting byte, packet to parse
  ///
  /// @return bool true if successfully parsed packet
  ///
  //bool parsePDM(int start_byte, const std::vector<uint8_t>& packet) override;
  
  ///
  /// Process performance degredation module (PDM) data from udp packets.
  ///
  /// @param[in] data pdm data array
  ///
  /// @return bool true if successful
  ///
  //bool processPDMData(const std::vector<uint8_t>& data) override;
  
  ///
  /// Parse packet into objects
  ///
  /// @param[in] start_byte starting byte, packet packet data to parse
  ///
  /// @return bool true if successfully parsed object data
  ///
  bool parseObjects(int start_byte, const std::vector<uint8_t>& packet) override;

  ///
  /// Process the object data from udp packets
  ///
  /// @param[in] data object data
  ///
  /// @return bool
  ///
  bool processObjectData(const std::vector<uint8_t>& data) override;

  ///
  /// Process the telemetry data from udp packets
  ///
  /// @param[in] data telemetry data
  ///
  /// @return bool
  ///
  bool processTelemetryData(const std::vector<uint8_t>& data) override;
  
  ///
  /// Process the slice data from udp packets
  ///
  /// @param[in] data slice data
  ///
  /// @return bool
  ///
  bool processSliceData(const std::vector<uint8_t>& data) override;
  
  ///
  cv::Mat initTransform(cv::Mat cameraMatrix, cv::Mat distCoeffs,
      int width, int height, bool radial);

  ///
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
  /// ROS node handler
  ros::NodeHandle node_handler_;

  /// Received packet bytes from HFL110
  int bytes_received_;

  /// Frame Header message
  std::shared_ptr<std_msgs::Header> frame_header_message_;

  /// PDM Header message
  std::shared_ptr<std_msgs::Header> pdm_header_message_;
  
  /// Object Header message
  std::shared_ptr<std_msgs::Header> object_header_message_;
  
  /// Telemetry Header message
  std::shared_ptr<std_msgs::Header> tele_header_message_;

  /// Slice Header message
  std::shared_ptr<std_msgs::Header> slice_header_message_;
  
  /// TF Header message
  std::shared_ptr<std_msgs::Header> tf_header_message_;

  /// Row and column Counter
  uint8_t row_, col_;

  /// Return counter
  uint8_t expected_packet_ = 0;

  /// Focal Length
  float focal_length_;

  // Camera info manager
  camera_info_manager::CameraInfoManager *camera_info_manager_;

  /// Pointer to depth image
  cv_bridge::CvImagePtr p_image_depth_;

  /// Pointer to 16 bit intensity image
  cv_bridge::CvImagePtr p_image_intensity_;

  /// Pointer to depth image second return
  cv_bridge::CvImagePtr p_image_depth2_;

  /// Pointer to 16 bit intensity image second return
  cv_bridge::CvImagePtr p_image_intensity2_;
  
  /// Pointer to crosstalk flags
  cv_bridge::CvImagePtr p_image_crosstalk_;
  
  /// Pointer to saturated flags
  cv_bridge::CvImagePtr p_image_saturated_;
  
  /// Pointer to superimposed flags
  cv_bridge::CvImagePtr p_image_superimposed_;
  
  /// Pointer to crosstalk2 flags
  cv_bridge::CvImagePtr p_image_crosstalk2_;
  
  /// Pointer to saturated2 flags
  cv_bridge::CvImagePtr p_image_saturated2_;
  
  /// Pointer to superimposed2 flags
  cv_bridge::CvImagePtr p_image_superimposed2_;
  
  /// Depth image publisher
  image_transport::CameraPublisher pub_depth_;

  /// Depth image publisher second return 2
  image_transport::CameraPublisher pub_depth2_;

  /// 16 bit Intensity image publisher
  image_transport::CameraPublisher pub_intensity_;

  /// 16 bit Intensity image publisher return 2
  image_transport::CameraPublisher pub_intensity2_;

  /// Crosstalk flag image publisher
  image_transport::CameraPublisher pub_ct_;
  
  /// Crosstalk2 flag image publisher
  image_transport::CameraPublisher pub_ct2_;
  
  /// Saturated flag image publisher
  image_transport::CameraPublisher pub_sat_;
  
  /// Saturated2 flag image publisher
  image_transport::CameraPublisher pub_sat2_;
  
  /// Superimposed flag image publisher
  image_transport::CameraPublisher pub_si_;
  
  /// Superimposed flag image publisher
  image_transport::CameraPublisher pub_si2_;

  /// Objects publisher
  ros::Publisher pub_objects_;
  
  /// Slices publisher
  ros::Publisher pub_slices_;
  
  /// Objects vector;
  std::vector<hflObj> objects_;

  /// Pointcloud publisher
  ros::Publisher pub_points_;

  /// Telemetry Data
  telemetry telem_{};

  /// Pointcloud msg
  std::shared_ptr<sensor_msgs::PointCloud2> pointcloud_;

  /// Slices msg
  std::shared_ptr<std_msgs::UInt16MultiArray> slices_;
  
  /// ROS Transform
  geometry_msgs::TransformStamped global_tf_;

  /// Transform
  cv::Mat transform_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

};
}  // namespace hfl
#endif  // IMAGE_PROCESSOR__HFL110DCU_H_
