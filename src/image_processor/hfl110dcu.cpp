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
/// @file hfl110dcu.cpp
///
/// @brief This file implements the hfl110dcu image processor class methods
///
#include "image_processor/hfl110dcu.h"
#include <pluginlib/class_list_macros.h>
#include <string>
#include <vector>
#include <cmath>

// Note: the initMatrix function in this file was originally written for
// the depth_image_proc ROS package and is included here instead. Full credit
// of development goes to the authors of that package and use of it in this
// package was granted by one of the authors since it is not a static function:
// https://github.com/ros-perception/image_pipeline/blob/
// 9b6764166096fa1d90706459fb70242f46ac8643/depth_image_proc/src/
// nodelets/point_cloud_xyzi_radial.cpp#L101

namespace hfl
{
HFL110DCU::HFL110DCU(std::string model, std::string version,
                     std::string frame_id, ros::NodeHandle& node_handler)
  : node_handler_(node_handler)
{
  // Set model and version
  model_ = model;
  version_ = version;

  // Initialize header messages
  frame_header_message_.reset(new std_msgs::Header());
  object_header_message_.reset(new std_msgs::Header());
  tf_header_message_.reset(new std_msgs::Header());

  ros::NodeHandle image_depth_nh(node_handler_, "depth");
  ros::NodeHandle image_intensity_16b_nh(node_handler_, "intensity");
  ros::NodeHandle image_depth2_nh(node_handler_, "depth2");
  ros::NodeHandle image_intensity2_16b_nh(node_handler_, "intensity2");
  ros::NodeHandle image_intensity_8b_nh(node_handler_, "intensity8");
  ros::NodeHandle objects_nh(node_handler_, "perception");

  image_transport::ImageTransport it_depth(image_depth_nh);
  image_transport::ImageTransport it_depth2(image_depth2_nh);
  image_transport::ImageTransport it_intensity_16b(image_intensity_16b_nh);
  image_transport::ImageTransport it_intensity2_16b(image_intensity2_16b_nh);
  image_transport::ImageTransport it_intensity_8b(image_intensity_8b_nh);

  // Initialize publishers
  pub_depth_ = it_depth.advertiseCamera("image_raw", 100);
  pub_intensity_ = it_intensity_16b.advertiseCamera("image_raw", 100);
  pub_depth2_ = it_depth2.advertiseCamera("image_raw", 100);
  pub_intensity2_ = it_intensity2_16b.advertiseCamera("image_raw", 100);
  pub_objects_ = objects_nh.advertise<visualization_msgs::MarkerArray>("objects", 100);
  pub_points_ = node_handler_.advertise<sensor_msgs::PointCloud2>("points", 1000);

  std::string default_calib_file = "~/.ros/camera_info/default.yaml";

  // Check camera info manager
  camera_info_manager_ =
    new camera_info_manager::CameraInfoManager(image_intensity_16b_nh, frame_id);

  // Initialize Message Headers
  frame_header_message_->frame_id = frame_id;
  frame_header_message_->seq = -1;
  object_header_message_->frame_id = "map";  // TODO(flynneva): make this a ROS parameter
  object_header_message_->seq = -1;
  tf_header_message_->frame_id = "map";
  tf_header_message_->seq = 0;
  global_tf_.child_frame_id = frame_id;
}

bool HFL110DCU::parseFrame(int start_byte, const std::vector<uint8_t>& packet)
{
  int byte_offset = 0;

  float range_1, range_2, temp_range = 0;
  uint16_t intensity_1, intensity_2 = 0;
  uint8_t type, ch = 0;

  // Build up range and intensity images
  for (col_ = 0; col_ < FRAME_COLUMNS; col_ += 1)
  {
    byte_offset = start_byte + (col_ * 4);
    // Populate range images
    range_1 = (global_offset_ + float(
      big_to_native(*reinterpret_cast<const uint16_t*>(&packet[byte_offset])))) / 256.0;
    range_2 = (global_offset_ + float(
      big_to_native(*reinterpret_cast<const uint16_t*>(&packet[byte_offset + 2])))) / 256.0;

    // Byte offset for intensity
    // Intensity Data follows Full Row of Depth Data (128 * 2 returns * 2bytes each)
    byte_offset = start_byte + 512 + (col_ * 4);

    // Populate intensity images
    intensity_1 = uint16_t(
      big_to_native(*reinterpret_cast<const uint16_t*>(&packet[byte_offset])));
    intensity_2 = uint16_t(
      big_to_native(*reinterpret_cast<const uint16_t*>(&packet[byte_offset + 2])));

    // If range is larger than 49m, set it to NAN
    if (range_1 > 49.0)
      range_1 = NAN;

    if (range_2 > 49.0)
      range_2 = NAN;

    // add in individual channel offsets
    if (ch == 0) {
      range_1 += ch1_offset_;
      range_2 += ch1_offset_;
    } else if (ch == 1) {
      range_1 += ch2_offset_;
      range_2 += ch2_offset_;
    } else if (ch == 2) {
      range_1 += ch3_offset_;
      range_2 += ch3_offset_;
    } else if (ch == 3) {
      range_1 += ch4_offset_;
      range_2 += ch4_offset_;
    }

    // add in individual intensity offsets
    if (intensity_1 < 500) {
      range_1 += int500_offset_;
    } else if (intensity_1 < 1000) {
      range_1 += int1000_offset_;
    } else if (intensity_1 < 1500) {
      range_1 += int1500_offset_;
    } else if (intensity_1 < 2000) {
      range_1 += int2000_offset_;
    } else if (intensity_1 < 2500) {
      range_1 += int2500_offset_;
    } else if (intensity_1 < 3000) {
      range_1 += int3000_offset_;
    } else if (intensity_1 < 3500) {
      range_1 += int3500_offset_;
    } else if (intensity_1 <= 4096) {
      range_1 += int4096_offset_;
    }

    if (intensity_2 < 500) {
      range_2 += int500_offset_;
    } else if (intensity_2 < 1000) {
      range_2 += int1000_offset_;
    } else if (intensity_2 < 1500) {
      range_2 += int1500_offset_;
    } else if (intensity_2 < 2000) {
      range_2 += int2000_offset_;
    } else if (intensity_2 < 2500) {
      range_2 += int2500_offset_;
    } else if (intensity_2 < 3000) {
      range_2 += int3000_offset_;
    } else if (intensity_2 < 3500) {
      range_2 += int3500_offset_;
    } else if (intensity_2 <= 4096) {
      range_2 += int4096_offset_;
    }

    p_image_depth_->image.at<float>(cv::Point(col_, row_)) = range_1;
    p_image_depth2_->image.at<float>(cv::Point(col_, row_)) = range_2;

    p_image_intensity_->image.at<uint16_t>(cv::Point(col_, row_)) = intensity_1;
    p_image_intensity2_->image.at<uint16_t>(cv::Point(col_, row_)) = intensity_2;

    // Byte offset for type flags
    // Type array indicates status of pixel
    byte_offset = start_byte + 1024 + col_;

    type = big_to_native(*reinterpret_cast<const uint8_t*>(&packet[byte_offset]));
    // ROS_INFO("OPTICAL_CT:      %i", ((type >> 4) & 1));
    // ROS_INFO("SUPERIMPOSED_CT: %i", ((type >> 3) & 1));
    // ROS_INFO("VALID_PULSE:     %i", ((type >> 2) & 1));
    // ROS_INFO("SATURATED:       %i", ((type >> 1) & 1));
    // ROS_INFO("CROSSTALK:       %i", ((type >> 0) & 1));
    // ROS_INFO("type offset: %i", byte_offset);
    if (ch < 3) {
      ch += 1;
    } else {
      ch = 0;
    }
  }

  return true;
}

bool HFL110DCU::processFrameData(const std::vector<uint8_t>& frame_data)
{
  if (version_ == "v1")
  {
    int size = frame_data.size();

    // identify packet by fragmentation offset
    row_ = FRAME_ROWS - 1 - big_to_native(*reinterpret_cast<const uint32_t*>(&frame_data[16]));
    int frame_num = big_to_native(*reinterpret_cast<const uint32_t*>(&frame_data[12]));

    // Check packet offset continuity
    if ( row_ != expected_packet_)
    {
      ROS_ERROR("Unexpected packet (dropped packet?) expecting: %i, received:  %i",
              expected_packet_, row_);
      expected_packet_ = FRAME_ROWS - 1;
      return false;
    }

    // First frame packet, reset frame data
    if (row_ == (FRAME_ROWS - 1))
    {
      // Set header message
      frame_header_message_->stamp = ros::Time::now();
      object_header_message_->stamp = frame_header_message_->stamp;
      tf_header_message_->stamp = frame_header_message_->stamp;

      // Set Pointcloud message
      pointcloud_.reset(new sensor_msgs::PointCloud2());
      pointcloud_->header = *frame_header_message_;
      pointcloud_->height = FRAME_ROWS * 2;
      pointcloud_->width = FRAME_COLUMNS * 2;

      sensor_msgs::PointCloud2Modifier modifier(*pointcloud_);
      modifier.setPointCloud2Fields(5,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "return", 1, sensor_msgs::PointField::UINT8);

      // Reset image pointers
      p_image_depth_.reset(new cv_bridge::CvImage);
      p_image_depth_->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      p_image_depth_->image = cv::Mat(FRAME_ROWS, FRAME_COLUMNS, CV_32FC1);

      p_image_intensity_.reset(new cv_bridge::CvImage);
      p_image_intensity_->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      p_image_intensity_->image = cv::Mat(FRAME_ROWS, FRAME_COLUMNS, CV_16UC1);

      p_image_depth2_.reset(new cv_bridge::CvImage);
      p_image_depth2_->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      p_image_depth2_->image = cv::Mat(FRAME_ROWS, FRAME_COLUMNS, CV_32FC1);

      p_image_intensity2_.reset(new cv_bridge::CvImage);
      p_image_intensity2_->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      p_image_intensity2_->image = cv::Mat(FRAME_ROWS, FRAME_COLUMNS, CV_16UC1);

      // Get intrinsic and extrinsic calibration parameters
      // CameraIntrinsics * camera_intrinsics;
      float fx = *reinterpret_cast<const float*>(&frame_data[20]);
      ROS_INFO_ONCE("fx: %.4f", fx);
      float fy = *reinterpret_cast<const float*>(&frame_data[24]);
      ROS_INFO_ONCE("fy: %.4f", fy);
      float ux = *reinterpret_cast<const float*>(&frame_data[28]);
      ROS_INFO_ONCE("ux: %.4f", ux);
      float uy = *reinterpret_cast<const float*>(&frame_data[32]);
      ROS_INFO_ONCE("uy: %.4f", uy);
      float r1 = *reinterpret_cast<const float*>(&frame_data[36]);
      ROS_INFO_ONCE("r1: %.4f", r1);
      float r2 = *reinterpret_cast<const float*>(&frame_data[40]);
      ROS_INFO_ONCE("r2: %.4f", r2);
      float t1 = *reinterpret_cast<const float*>(&frame_data[44]);
      ROS_INFO_ONCE("t1: %.4f", t1);
      float t2 = *reinterpret_cast<const float*>(&frame_data[48]);
      ROS_INFO_ONCE("t2: %.4f", t2);
      float r4 = *reinterpret_cast<const float*>(&frame_data[52]);
      ROS_INFO_ONCE("r4: %.4f", r4);

      float intrinsic_yaw = *reinterpret_cast<const float*>(&frame_data[56]);
      float intrinsic_pitch = *reinterpret_cast<const float*>(&frame_data[60]);
      float extrinsic_yaw = *reinterpret_cast<const float*>(&frame_data[64]) - 1.5708;
      float extrinsic_pitch = *reinterpret_cast<const float*>(&frame_data[68]);
      float extrinsic_roll = *reinterpret_cast<const float*>(&frame_data[72]) - 1.5708;
      float extrinsic_z = *reinterpret_cast<const float*>(&frame_data[76]);
      float extrinsic_y = *reinterpret_cast<const float*>(&frame_data[80]);
      float extrinsic_x = *reinterpret_cast<const float*>(&frame_data[84]);

      // set extrinsics to global tf
      tf2::Quaternion q;
      global_tf_.transform.translation.x = extrinsic_x;
      global_tf_.transform.translation.y = extrinsic_y;
      global_tf_.transform.translation.z = extrinsic_z;
      q.setRPY(extrinsic_roll, extrinsic_pitch, extrinsic_yaw);
      global_tf_.transform.rotation = tf2::toMsg(q);  // q.x();

      // check camera info manager
      if (camera_info_manager_ != NULL)
      {
        auto ci = camera_info_manager_->getCameraInfo();

        if (ci.K[0] != fx)
        {
          ROS_WARN("Initialized intrinsics do not match those received from sensor");
          ROS_WARN("Setting intrinsics to values received from sensor");
          // set default values
          ci.distortion_model = "rational_polynomial";
          ci.height = FRAME_ROWS;
          ci.width = FRAME_COLUMNS;
          ci.D.resize(8);
          ci.D[0] = r1;
          ci.D[1] = r2;
          ci.D[2] = t1;
          ci.D[3] = t2;
          ci.D[4] = 0;
          ci.D[5] = r4;
          ci.D[6] = 0;
          ci.D[7] = 0;

          ci.K[0] = fx;
          ci.K[2] = ux;
          ci.K[4] = fy;
          ci.K[5] = uy;
          ci.K[8] = 1;

          ci.P[0] = fx;
          ci.P[2] = ux;
          ci.P[4] = fy;
          ci.P[5] = uy;
          ci.P[11] = 1;

          camera_info_manager_->setCameraInfo(ci);

          transform_ = initTransform(cv::Mat_<double>(3, 3, &ci.K[0]),
                                         cv::Mat(ci.D), ci.width, ci.height, true);
        }
      }
    }

    // Parse image data
    parseFrame(92, frame_data);

    // Last frame packet, pulish frame data
    if (row_ == 0)
    {
      // Get camera info
      auto ci = camera_info_manager_->getCameraInfo();
      sensor_msgs::CameraInfoPtr flash_cam_info(new sensor_msgs::CameraInfo(ci));

      flash_cam_info->header = *frame_header_message_;

      p_image_depth_->header = *frame_header_message_;
      p_image_intensity_->header = *frame_header_message_;
      pub_depth_.publish(p_image_depth_->toImageMsg(), flash_cam_info);
      pub_intensity_.publish(p_image_intensity_->toImageMsg(), flash_cam_info);

      p_image_depth2_->header = *frame_header_message_;
      p_image_intensity2_->header = *frame_header_message_;
      pub_depth2_.publish(p_image_depth2_->toImageMsg(), flash_cam_info);
      pub_intensity2_.publish(p_image_intensity2_->toImageMsg(), flash_cam_info);

      // iterators
      sensor_msgs::PointCloud2Iterator<float> out_x(*pointcloud_, "x");
      sensor_msgs::PointCloud2Iterator<float> out_y(*pointcloud_, "y");
      sensor_msgs::PointCloud2Iterator<float> out_z(*pointcloud_, "z");
      sensor_msgs::PointCloud2Iterator<float> out_i(*pointcloud_, "intensity");
      sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*pointcloud_, "return");

      // loop through rows and cols
      for (row_ = 0; row_ < FRAME_ROWS; row_ += 1)
      {
        for (col_ = 0; col_ < FRAME_COLUMNS; col_ += 1)
        {
          // Return 1
          const cv::Vec3f &cvPoint =
              transform_.at<cv::Vec3f>(col_, row_) *
                    p_image_depth_->image.at<float>(cv::Point(col_, row_));

          *out_x = cvPoint(0);
          *out_y = cvPoint(1);
          *out_z = cvPoint(2);
          *out_i = p_image_intensity_->image.at<uint16_t>(cv::Point(col_, row_));
          *out_r = 1;

          out_x += 1;
          out_y += 1;
          out_z += 1;
          out_i += 1;
          out_r += 1;

          // Return 2
          const cv::Vec3f &cvPoint2 = transform_.at<cv::Vec3f>(col_, row_) *
                     p_image_depth2_->image.at<float>(cv::Point(col_, row_));

          *out_x = cvPoint2(0);
          *out_y = cvPoint2(1);
          *out_z = cvPoint2(2);
          *out_i = p_image_intensity2_->image.at<uint16_t>(cv::Point(col_, row_));
          *out_r = 2;

          out_x += 1;
          out_y += 1;
          out_z += 1;
          out_i += 1;
          out_r += 1;
        }
      }

      // publish transform
      static tf2_ros::TransformBroadcaster br;
      global_tf_.header = *tf_header_message_;
      br.sendTransform(global_tf_);

      // publish pointcloud
      pub_points_.publish(*pointcloud_);
    }
    expected_packet_ = (expected_packet_ > 0)? expected_packet_ - 1: FRAME_ROWS - 1;
  }
  return true;
}

bool HFL110DCU::parseObjects(int start_byte, const std::vector<uint8_t>& packet)
{
  int count = objects_.size();
  int last_object = 0;
  if (count == 0) {
    // first packet, stop after 11 objects
    last_object = 11;
  } else if (count == 11) {
    // second packet, stop after 20 objects
    last_object = 20;
  }

  for (int i = start_byte; i < packet.size(); i += 129)
  {
    if (count == last_object)
    {
      break;
    }
    // create new object
    objects_.push_back(hflObj());
    // object geometry attributes
    objects_[count].geometry.x_rear_r = *reinterpret_cast<const float*>(&packet[i + 0]);
    objects_[count].geometry.y_rear_r = *reinterpret_cast<const float*>(&packet[i + 4]);
    objects_[count].geometry.x_rear_l = *reinterpret_cast<const float*>(&packet[i + 8]);
    objects_[count].geometry.y_rear_l = *reinterpret_cast<const float*>(&packet[i + 12]);
    objects_[count].geometry.x_front_l = *reinterpret_cast<const float*>(&packet[i + 16]);
    objects_[count].geometry.y_front_l = *reinterpret_cast<const float*>(&packet[i + 20]);
    objects_[count].geometry.height = *reinterpret_cast<const float*>(&packet[i + 24]);
    objects_[count].geometry.ground_offset = *reinterpret_cast<const float*>(&packet[i + 28]);
    objects_[count].geometry.fDistX = *reinterpret_cast<const float*>(&packet[i + 32]);
    objects_[count].geometry.fDistY = *reinterpret_cast<const float*>(&packet[i + 36]);
    objects_[count].geometry.yaw = *reinterpret_cast<const float*>(&packet[i + 40]);
    // object kinematic attributes
    objects_[count].kinematics.fVabsX = *reinterpret_cast<const float*>(&packet[i + 44]);
    objects_[count].kinematics.fVabsY = *reinterpret_cast<const float*>(&packet[i + 48]);
    objects_[count].kinematics.fVrelX = *reinterpret_cast<const float*>(&packet[i + 52]);
    objects_[count].kinematics.fVrelY = *reinterpret_cast<const float*>(&packet[i + 56]);
    objects_[count].kinematics.fAabsX = *reinterpret_cast<const float*>(&packet[i + 60]);
    objects_[count].kinematics.fDistXDistY = *reinterpret_cast<const float*>(&packet[i + 64]);
    objects_[count].kinematics.fDistXVx = *reinterpret_cast<const float*>(&packet[i + 68]);
    objects_[count].kinematics.fDistXVy = *reinterpret_cast<const float*>(&packet[i + 72]);
    objects_[count].kinematics.fDistXAx = *reinterpret_cast<const float*>(&packet[i + 76]);
    objects_[count].kinematics.fDistXAy = *reinterpret_cast<const float*>(&packet[i + 80]);
    objects_[count].kinematics.fDistYVx = *reinterpret_cast<const float*>(&packet[i + 84]);
    objects_[count].kinematics.fDistYVy = *reinterpret_cast<const float*>(&packet[i + 88]);
    objects_[count].kinematics.fDistYAx = *reinterpret_cast<const float*>(&packet[i + 92]);
    objects_[count].kinematics.fDistYAy = *reinterpret_cast<const float*>(&packet[i + 96]);
    objects_[count].kinematics.fVxVy = *reinterpret_cast<const float*>(&packet[i + 100]);
    objects_[count].kinematics.fVxAx = *reinterpret_cast<const float*>(&packet[i + 104]);
    objects_[count].kinematics.fVxAy = *reinterpret_cast<const float*>(&packet[i + 108]);
    objects_[count].kinematics.fVyAx = *reinterpret_cast<const float*>(&packet[i + 112]);
    objects_[count].kinematics.fVyAy = *reinterpret_cast<const float*>(&packet[i + 116]);
    objects_[count].kinematics.fAxAy = *reinterpret_cast<const float*>(&packet[i + 120]);
    // object state
    objects_[count].state = *reinterpret_cast<const hfl::objState*>(&packet[i + 124]);
    objects_[count].dynamic_props = *reinterpret_cast<const hfl::objDyn*>(&packet[i + 125]);
    objects_[count].quality = *reinterpret_cast<const uint8_t*>(&packet[i + 126]);
    // object classification attributes
    objects_[count].classification = *reinterpret_cast<const uint8_t*>(&packet[i + 127]);
    objects_[count].confidence = *reinterpret_cast<const uint8_t*>(&packet[i + 128]);
    count += 1;
  }

  return true;
}

bool HFL110DCU::processObjectData(const std::vector<uint8_t>& object_data)
{
  // grab the time when recieved packet
  object_header_message_->stamp = ros::Time::now();
  object_header_message_->seq += 1;

  // identify packet by fragmentation offset
  uint32_t obj_packet =
      (big_to_native(*reinterpret_cast<const uint32_t*>(&object_data[10])) >> 0) & 1;

  parseObjects(14, object_data);

  if (obj_packet == 1)
  {
    visualization_msgs::Marker bBox;
    visualization_msgs::MarkerArray marker_array;
    tf2::Quaternion q;

    for (int i = 0; i < objects_.size(); i += 1)
    {
      bBox.pose.position.x = (objects_[i].geometry.x_rear_r + 0.5 *
                             (objects_[i].geometry.x_front_l - objects_[i].geometry.x_rear_r)) +
                             objects_[i].geometry.fDistX;
      bBox.pose.position.y = (objects_[i].geometry.y_rear_r + 0.5 *
                             (objects_[i].geometry.y_front_l - objects_[i].geometry.y_rear_r)) +
                             objects_[i].geometry.fDistY;
      bBox.pose.position.z =
        objects_[i].geometry.ground_offset + (objects_[i].geometry.height / 2.0);

      q.setRPY(0, 0, objects_[i].geometry.yaw);
      bBox.pose.orientation = tf2::toMsg(q);

      float length = sqrt((objects_[i].geometry.x_front_l - objects_[i].geometry.x_rear_l) *
                          (objects_[i].geometry.x_front_l - objects_[i].geometry.x_rear_l) +
                          (objects_[i].geometry.y_front_l - objects_[i].geometry.y_rear_l) *
                          (objects_[i].geometry.y_front_l - objects_[i].geometry.y_rear_l));
      float width = sqrt((objects_[i].geometry.x_rear_r - objects_[i].geometry.x_rear_l) *
                         (objects_[i].geometry.x_rear_r - objects_[i].geometry.x_rear_l) +
                         (objects_[i].geometry.y_rear_r - objects_[i].geometry.y_rear_l) *
                         (objects_[i].geometry.y_rear_r - objects_[i].geometry.y_rear_l));
      bBox.scale.x = length;
      bBox.scale.y = width;
      bBox.scale.z = objects_[i].geometry.height + objects_[i].geometry.ground_offset;

      // set color from classification
      if (objects_[i].classification == 9)
      {
        // TL
        bBox.color.r = 240.0 / 255.0;
        bBox.color.g = 230.0 / 255.0;
        bBox.color.b = 140.0 / 255.0;
        bBox.color.a = objects_[i].confidence / 100.0;
      } else if (objects_[i].classification == 8) {
        // OTHER VEHICLE
        bBox.color.r = 238.0 / 255.0;
        bBox.color.g = 232.0 / 255.0;
        bBox.color.b = 170.0 / 255.0;
        bBox.color.a = objects_[i].confidence / 100.0;
      } else if (objects_[i].classification == 7) {
        // UNCLASSIFIED
        bBox.color.r = 238.0 / 255.0;
        bBox.color.g = 232.0 / 255.0;
        bBox.color.b = 170.0 / 255.0;
        bBox.color.a = objects_[i].confidence / 100.0;
      } else if (objects_[i].classification == 6) {
        // WIDE
        bBox.color.r = 238.0 / 255.0;
        bBox.color.g = 232.0 / 255.0;
        bBox.color.b = 170.0 / 255.0;
        bBox.color.a = objects_[i].confidence / 100.0;
      } else if (objects_[i].classification == 5) {
        // BICYCLE
        bBox.color.r = 255.0 / 255.0;
        bBox.color.g = 140.0 / 255.0;
        bBox.color.b = 0;
        bBox.color.a = objects_[i].confidence / 100.0;
      } else if (objects_[i].classification == 4) {
        // MOTORCYCLE
        bBox.color.r = 230 / 255.0;
        bBox.color.g = 190 / 255.0;
        bBox.color.b = 138 / 255.0;
        bBox.color.a = objects_[i].confidence / 100.0;
      } else if (objects_[i].classification == 3) {
        // PERSON
        bBox.color.r = 215 / 255.0;
        bBox.color.g = 215 / 255.0;
        bBox.color.b = 0;
        bBox.color.a = objects_[i].confidence / 100.0;
      } else if (objects_[i].classification == 2) {
        // TRUCK
        bBox.color.r = 218 / 255.0;
        bBox.color.g = 165 / 255.0;
        bBox.color.b = 32 / 255.0;
        bBox.color.a = objects_[i].confidence / 100.0;
      } else if (objects_[i].classification == 1) {
        // CAR
        bBox.color.r = 139 / 255.0;
        bBox.color.g = 69 / 255.0;
        bBox.color.b = 19 / 255.0;
        bBox.color.a = objects_[i].confidence / 100.0;
      } else if (objects_[i].classification == 0) {
        // POINT
        bBox.color.r = 210 / 255.0;
        bBox.color.g = 105 / 255.0;
        bBox.color.b = 30 / 255.0;
        bBox.color.a = objects_[i].confidence / 100.0;
      }

      bBox.type = 1;
      bBox.id = i;
      bBox.lifetime = ros::Duration();
      bBox.frame_locked = false;
      // bBox.text = ("OBJECT%i", i);
      bBox.action = visualization_msgs::Marker::ADD;
      bBox.header = *object_header_message_;

      marker_array.markers.push_back(bBox);
    }
    pub_objects_.publish(marker_array);
    objects_.clear();
  }

  return true;
}

cv::Mat HFL110DCU::initTransform(cv::Mat cameraMatrix, cv::Mat distCoeffs,
                                     int width, int height, bool radial)
{
  int i, j;
  int totalsize = width*height;
  cv::Mat pixelVectors(1, totalsize, CV_32FC3);
  cv::Mat dst(1, totalsize, CV_32FC3);

  cv::Mat sensorPoints(cv::Size(height, width), CV_32FC2);
  cv::Mat undistortedSensorPoints(1, totalsize, CV_32FC2);

  std::vector<cv::Mat> ch;
  for(j = 0; j < height; j++)
  {
    for(i = 0; i < width; i++)
    {
      cv::Vec2f &p = sensorPoints.at<cv::Vec2f>(i, j);
      p[0] = i;
      p[1] = j;
    }
  }

  sensorPoints = sensorPoints.reshape(2, 1);

  cv::undistortPoints(sensorPoints, undistortedSensorPoints, cameraMatrix, distCoeffs);

  ch.push_back(undistortedSensorPoints);
  ch.push_back(cv::Mat::ones(1, totalsize, CV_32FC1));
  cv::merge(ch, pixelVectors);

  if(radial)
  {
    for(i = 0; i < totalsize; i++)
    {
      normalize(pixelVectors.at<cv::Vec3f>(i),
      dst.at<cv::Vec3f>(i));
    }
    pixelVectors = dst;
  }
  return pixelVectors.reshape(3, width);
}
}  // namespace hfl
