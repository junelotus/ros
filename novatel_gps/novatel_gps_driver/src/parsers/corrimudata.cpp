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

#include <novatel_gps_driver/parsers/corrimudata.h>

#include <novatel_gps_driver/parsers/header.h>
#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <sstream>

namespace novatel_gps_driver {

const std::string novatel_gps_driver::CorrImuDataParser::MESSAGE_NAME =
    "CORRIMUDATA";

uint32_t novatel_gps_driver::CorrImuDataParser::GetMessageId() const {
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::CorrImuDataParser::GetMessageName()
    const {
  return MESSAGE_NAME;
}

novatel_gps_msgs::NovatelCorrectedImuDataPtr
novatel_gps_driver::CorrImuDataParser::ParseBinary(
    const novatel_gps_driver::BinaryMessage& bin_msg) throw(ParseException) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected corrimudata message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::NovatelCorrectedImuDataPtr ros_msg =
      boost::make_shared<novatel_gps_msgs::NovatelCorrectedImuData>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = "CORRIMUDATA";

  ros_msg->gps_week_num = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->gps_seconds = ParseDouble(&bin_msg.data_[4]);
  ros_msg->pitch_rate = ParseDouble(&bin_msg.data_[12]);
  ros_msg->roll_rate = ParseDouble(&bin_msg.data_[20]);
  ros_msg->yaw_rate = ParseDouble(&bin_msg.data_[28]);
  ros_msg->lateral_acceleration = ParseDouble(&bin_msg.data_[36]);
  ros_msg->longitudinal_acceleration = ParseDouble(&bin_msg.data_[44]);
  ros_msg->vertical_acceleration = ParseDouble(&bin_msg.data_[52]);
#if 0
  ROS_DEBUG("CorrImuDataParser::ParseBinary:%d,%f,%f,%f,%f,%f,%f,%f",ros_msg->gps_week_num,ros_msg->gps_seconds,ros_msg->pitch_rate,
    ros_msg->roll_rate,ros_msg->yaw_rate,ros_msg->lateral_acceleration,ros_msg->longitudinal_acceleration,ros_msg->vertical_acceleration);
#endif
  return ros_msg;
}

novatel_gps_msgs::NovatelCorrectedImuDataPtr
novatel_gps_driver::CorrImuDataParser::ParseAscii(
    const novatel_gps_driver::NovatelSentence& sentence) throw(ParseException) {
  if (sentence.body.size() != ASCII_FIELDS) {
    std::stringstream error;
    error << "Unexpected number of fields in CORRIMUDATA log: "
          << sentence.body.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::NovatelCorrectedImuDataPtr msg =
      boost::make_shared<novatel_gps_msgs::NovatelCorrectedImuData>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  valid &= ParseUInt32(sentence.body[0], msg->gps_week_num);
  valid &= ParseDouble(sentence.body[1], msg->gps_seconds);
  valid &= ParseDouble(sentence.body[2], msg->pitch_rate);
  valid &= ParseDouble(sentence.body[3], msg->roll_rate);
  valid &= ParseDouble(sentence.body[4], msg->yaw_rate);
  valid &= ParseDouble(sentence.body[5], msg->lateral_acceleration);
  valid &= ParseDouble(sentence.body[6], msg->longitudinal_acceleration);
  valid &= ParseDouble(sentence.body[7], msg->vertical_acceleration);

  if (!valid) {
    throw ParseException("Error parsing CORRIMUDATA log.");
  }

  return msg;
}

#if 1

novatel_gps_msgs::CORRIMUDATA1Ptr
novatel_gps_driver::CorrImuDataParser::ParseBinary1(
    const novatel_gps_driver::BinaryMessage& bin_msg) {
#if 0
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected corrimudata message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
#endif
  novatel_gps_msgs::CORRIMUDATA1Ptr ros_msg =
      boost::make_shared<novatel_gps_msgs::CORRIMUDATA1>();

  HeaderParser h_parser;

  ros_msg->header = h_parser.ParseBinary1(bin_msg);

  ros_msg->gps_week = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->gps_seconds = ParseDouble(&bin_msg.data_[4]);
  ros_msg->pitch_rate = ParseDouble(&bin_msg.data_[12]);
  ros_msg->roll_rate = ParseDouble(&bin_msg.data_[20]);
  ros_msg->yaw_rate = ParseDouble(&bin_msg.data_[28]);
  ros_msg->x_accel = ParseDouble(&bin_msg.data_[36]);
  ros_msg->y_accel = ParseDouble(&bin_msg.data_[44]);
  ros_msg->z_accel = ParseDouble(&bin_msg.data_[52]);

#if 0
  ROS_DEBUG("CorrImuDataParser::ParseBinary1:%d,%f,%f,%f,%f,%f,%f,%f",ros_msg->week,ros_msg->gpssec,ros_msg->pitch_rate,
    ros_msg->roll_rate,ros_msg->yaw_rate,ros_msg->x_accel,ros_msg->y_accel,ros_msg->z_accel);
#endif

  return ros_msg;
}

novatel_gps_msgs::CORRIMUDATA1Ptr
novatel_gps_driver::CorrImuDataParser::ParseAscii1(
    const novatel_gps_driver::NovatelSentence& sentence) {
#if 0
  if (sentence.body.size() != ASCII_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of fields in CORRIMUDATA log: " << sentence.body.size();
    throw ParseException(error.str());
  }
#endif
  novatel_gps_msgs::CORRIMUDATA1Ptr msg =
      boost::make_shared<novatel_gps_msgs::CORRIMUDATA1>();

  HeaderParser h_parser;
  msg->header = h_parser.ParseAscii1(sentence);

  bool valid = true;

  valid &= ParseInt32(sentence.body[0], msg->gps_week);
  valid &= ParseDouble(sentence.body[1], msg->gps_seconds);
  valid &= ParseDouble(sentence.body[2], msg->pitch_rate);
  valid &= ParseDouble(sentence.body[3], msg->roll_rate);
  valid &= ParseDouble(sentence.body[4], msg->yaw_rate);
  valid &= ParseDouble(sentence.body[5], msg->x_accel);
  valid &= ParseDouble(sentence.body[6], msg->y_accel);
  valid &= ParseDouble(sentence.body[7], msg->z_accel);

  return msg;
}
}

#endif
