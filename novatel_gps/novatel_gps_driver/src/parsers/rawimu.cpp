// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <novatel_gps_driver/parsers/rawimu.h>

#include <novatel_gps_driver/parsers/header.h>

#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <sstream>

namespace novatel_gps_driver {
const std::string RawImuParser::MESSAGE_NAME = "RAWIMUA";

uint32_t RawImuParser::GetMessageId() const { return MESSAGE_ID; }

const std::string RawImuParser::GetMessageName() const { return MESSAGE_NAME; }

novatel_gps_msgs::RAWIMU1Ptr RawImuParser::ParseBinary1(
    const BinaryMessage& bin_msg) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected RAWIMU message length: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::RAWIMU1Ptr ros_msg =
      boost::make_shared<novatel_gps_msgs::RAWIMU1>();
  HeaderParser header_parser;

  ros_msg->header = header_parser.ParseBinary1(bin_msg);

  ros_msg->gps_week = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->gps_seconds = ParseDouble(&bin_msg.data_[4]);

  ros_msg->imu_status = ParseUInt32(&bin_msg.data_[12]);
  ros_msg->z_velocity_change = ParseInt32(&bin_msg.data_[16]);
  ros_msg->y_velocity_change_neg = ParseInt32(&bin_msg.data_[20]);
  ros_msg->x_velocity_change = ParseInt32(&bin_msg.data_[24]);

  ros_msg->z_angle_change = ParseInt32(&bin_msg.data_[28]);
  ros_msg->y_angle_change_neg = ParseInt32(&bin_msg.data_[32]);
  ros_msg->x_angle_change = ParseInt32(&bin_msg.data_[36]);  //////

//  ROS_INFO("ImuDataParser::ParseBinary1:%d,%f,%d,%d,%d,%d,%d,%d,%d",ros_msg->gps_week,ros_msg->gps_seconds,ros_msg->imu_status,
//    ros_msg->z_velocity_change,ros_msg->y_velocity_change_neg,ros_msg->x_velocity_change,ros_msg->z_angle_change,ros_msg->y_angle_change_neg, ros_msg->x_angle_change);

  return ros_msg;
}

novatel_gps_msgs::RAWIMU1Ptr RawImuParser::ParseAscii1(
    const NovatelSentence& sentence) {
  novatel_gps_msgs::RAWIMU1Ptr msg =
      boost::make_shared<novatel_gps_msgs::RAWIMU1>();
  HeaderParser h_parser;
  msg->header = h_parser.ParseAscii1(sentence);

  bool valid = true;

  valid = valid && ParseUInt32(sentence.body[0], msg->gps_week);
  valid = valid && ParseDouble(sentence.body[1], msg->gps_seconds);
  valid = valid && ParseHexUint32(sentence.body[2], msg->imu_status);
  valid = valid && ParseInt32(sentence.body[3], msg->z_velocity_change);
  valid = valid && ParseInt32(sentence.body[4], msg->y_velocity_change_neg);
  valid = valid && ParseInt32(sentence.body[5], msg->x_velocity_change);
  valid = valid && ParseInt32(sentence.body[6], msg->z_angle_change);
  valid = valid && ParseInt32(sentence.body[7], msg->y_angle_change_neg);
  valid = valid && ParseInt32(sentence.body[8], msg->x_angle_change);

//  ROS_INFO("ImuDataParser::ParseAscii1:%d,%f,%d,%d,%d,%d,%d,%d,%d",msg->gps_week,msg->gps_seconds,msg->imu_status,
//    msg->z_velocity_change,msg->y_velocity_change_neg,msg->x_velocity_change,msg->z_angle_change,msg->y_angle_change_neg, msg->x_angle_change);
  return msg;
}
}
