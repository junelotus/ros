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

#include <novatel_gps_driver/parsers/bestpos.h>

#include <novatel_gps_driver/parsers/header.h>

#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <sstream>

namespace novatel_gps_driver {
const std::string BestposParser::MESSAGE_NAME = "BESTPOS";

uint32_t BestposParser::GetMessageId() const { return MESSAGE_ID; }

const std::string BestposParser::GetMessageName() const { return MESSAGE_NAME; }

novatel_gps_msgs::NovatelPositionPtr BestposParser::ParseBinary(
    const BinaryMessage& bin_msg) throw(ParseException) {
  if (bin_msg.data_.size() != BINARY_LENGTH) {
    std::stringstream error;
    error << "Unexpected BESTPOS message length: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::NovatelPositionPtr ros_msg =
      boost::make_shared<novatel_gps_msgs::NovatelPosition>();
  HeaderParser header_parser;
  ros_msg->novatel_msg_header = header_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = MESSAGE_NAME;

  uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
  if (solution_status > MAX_SOLUTION_STATUS) {
    std::stringstream error;
    error << "Unknown solution status: " << solution_status;
    throw ParseException(error.str());
  }
  ros_msg->solution_status = SOLUTION_STATUSES[solution_status];
  uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
  if (pos_type > MAX_POSITION_TYPE) {
    std::stringstream error;
    error << "Unknown position type: " << pos_type;
    throw ParseException(error.str());
  }
  ros_msg->position_type = POSITION_TYPES[pos_type];
  ros_msg->lat = ParseDouble(&bin_msg.data_[8]);
  ros_msg->lon = ParseDouble(&bin_msg.data_[16]);
  ros_msg->height = ParseDouble(&bin_msg.data_[24]);
  ros_msg->undulation = ParseFloat(&bin_msg.data_[32]);
  uint16_t datum_id = ParseUInt16(&bin_msg.data_[36]);
  if (datum_id > MAX_DATUM) {
    std::stringstream error;
    error << "Unknown datum: " << datum_id;
    throw ParseException(error.str());
  }
  ros_msg->datum_id = DATUMS[datum_id];
  ros_msg->lat_sigma = ParseFloat(&bin_msg.data_[40]);
  ros_msg->lon_sigma = ParseFloat(&bin_msg.data_[44]);
  ros_msg->height_sigma = ParseFloat(&bin_msg.data_[48]);
  ros_msg->base_station_id.resize(4);
  std::copy(&bin_msg.data_[52], &bin_msg.data_[56],
            &ros_msg->base_station_id[0]);
  ros_msg->diff_age = ParseFloat(&bin_msg.data_[56]);
  ros_msg->solution_age = ParseFloat(&bin_msg.data_[60]);
  ros_msg->num_satellites_tracked = bin_msg.data_[64];
  ros_msg->num_satellites_used_in_solution = bin_msg.data_[65];
  ros_msg->num_gps_and_glonass_l1_used_in_solution = bin_msg.data_[66];
  ros_msg->num_gps_and_glonass_l1_and_l2_used_in_solution = bin_msg.data_[67];
  GetExtendedSolutionStatusMessage(bin_msg.data_[69],
                                   ros_msg->extended_solution_status);
  GetSignalsUsed(bin_msg.data_[70], ros_msg->signal_mask);

#if 0
    ROS_DEBUG("ParseBinary data:%f,%f,%f,%f,%d,%f,%f,%f,%f,%f",
      ros_msg->lat,ros_msg->lon,ros_msg->height,
               ros_msg->undulation,datum_id,ros_msg->lat_sigma,ros_msg->lon_sigma,ros_msg->height_sigma,
               ros_msg->diff_age,ros_msg->solution_age);
#endif
  return ros_msg;
}

novatel_gps_msgs::NovatelPositionPtr BestposParser::ParseAscii(
    const NovatelSentence& sentence) throw(ParseException) {
  novatel_gps_msgs::NovatelPositionPtr msg =
      boost::make_shared<novatel_gps_msgs::NovatelPosition>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  if (sentence.body.size() != ASCII_LENGTH) {
    std::stringstream error;
    error << "Unexpected number of BESTPOS message fields: "
          << sentence.body.size();
    throw ParseException(error.str());
  }

  bool valid = true;

  msg->solution_status = sentence.body[0];
  msg->position_type = sentence.body[1];
  valid = valid && ParseDouble(sentence.body[2], msg->lat);
  valid = valid && ParseDouble(sentence.body[3], msg->lon);
  valid = valid && ParseDouble(sentence.body[4], msg->height);
  valid = valid && ParseFloat(sentence.body[5], msg->undulation);
  msg->datum_id = sentence.body[6];
  valid = valid && ParseFloat(sentence.body[7], msg->lat_sigma);
  valid = valid && ParseFloat(sentence.body[8], msg->lon_sigma);
  valid = valid && ParseFloat(sentence.body[9], msg->height_sigma);
  msg->base_station_id = sentence.body[10];
  valid = valid && ParseFloat(sentence.body[11], msg->diff_age);
  valid = valid && ParseFloat(sentence.body[12], msg->solution_age);
  valid = valid && ParseUInt8(sentence.body[13], msg->num_satellites_tracked);
  valid = valid &&
          ParseUInt8(sentence.body[14], msg->num_satellites_used_in_solution);
  valid = valid && ParseUInt8(sentence.body[15],
                              msg->num_gps_and_glonass_l1_used_in_solution);
  valid =
      valid && ParseUInt8(sentence.body[16],
                          msg->num_gps_and_glonass_l1_and_l2_used_in_solution);

  // skip reserved field
  uint32_t extended_solution_status = 0;
  valid = valid && ParseUInt32(sentence.body[18], extended_solution_status, 16);
  GetExtendedSolutionStatusMessage(extended_solution_status,
                                   msg->extended_solution_status);

  // skip reserved field
  uint32_t signal_mask = 0;
  valid = valid && ParseUInt32(sentence.body[20], signal_mask, 16);
  GetSignalsUsed(signal_mask, msg->signal_mask);

  if (!valid) {
    throw ParseException("Invalid field in BESTPOS message");
  }

#if 0
    ROS_DEBUG("ParseAscii data:%f,%f,%f,%f,%f,%f,%f",
      msg->lat,msg->lon,msg->height,
               msg->undulation,msg->lat_sigma,msg->lon_sigma,msg->height_sigma
               );
#endif
  return msg;
}

/*************
 *
 *
 *new add by cgl
 *
 *
 *************/

#if 1

novatel_gps_msgs::BESTPOS1Ptr BestposParser::ParseBinary1(
    const BinaryMessage& bin_msg) {
#if 0
    if (bin_msg.data_.size() != BINARY_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected BESTPOS message length: " << bin_msg.data_.size();
      throw ParseException(error.str());
    }
#endif
  novatel_gps_msgs::BESTPOS1Ptr ros_msg =
      boost::make_shared<novatel_gps_msgs::BESTPOS1>();

  HeaderParser header_parser;

  ros_msg->header = header_parser.ParseBinary1(bin_msg);

  ros_msg->solution_status = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->position_type = ParseUInt32(&bin_msg.data_[4]);  //////

  ros_msg->latitude = ParseDouble(&bin_msg.data_[8]);    //////
  ros_msg->longitude = ParseDouble(&bin_msg.data_[16]);  //////
  ros_msg->altitude = ParseDouble(&bin_msg.data_[24]);   //////

  ros_msg->undulation = ParseFloat(&bin_msg.data_[32]);  //////
  ros_msg->datum_id = ParseUInt16(&bin_msg.data_[36]);

  ros_msg->latitude_std = ParseFloat(&bin_msg.data_[40]);   //////
  ros_msg->longitude_std = ParseFloat(&bin_msg.data_[44]);  //////
  ros_msg->altitude_std = ParseFloat(&bin_msg.data_[48]);   //////
  std::copy(&bin_msg.data_[52], &bin_msg.data_[56], &ros_msg->stn_id[0]);
  ros_msg->diff_age = ParseFloat(&bin_msg.data_[56]);
  ros_msg->sol_age = ParseFloat(&bin_msg.data_[60]);

#if 0
  ROS_DEBUG("ParseBinary1 data:%f,%f,%f,%f,%d,%f,%f,%f,%f,%f",
      ros_msg->latitude,ros_msg->longitude,ros_msg->altitude,
               ros_msg->undulation,ros_msg->datum_id,ros_msg->latitude_std,ros_msg->longitude_std,ros_msg->altitude_std,
               ros_msg->diff_age,ros_msg->sol_age);
#endif
  return ros_msg;
}

novatel_gps_msgs::BESTPOS1Ptr BestposParser::ParseAscii1(
    const NovatelSentence& sentence) {
  novatel_gps_msgs::BESTPOS1Ptr msg =
      boost::make_shared<novatel_gps_msgs::BESTPOS1>();
  HeaderParser h_parser;
  msg->header = h_parser.ParseAscii1(sentence);
#if 0
    if (sentence.body.size() != ASCII_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected number of BESTPOS message fields: " << sentence.body.size();
      throw ParseException(error.str());
    }
#endif

  bool valid = true;

  int16_t solution_status = -1;

#if 0
    for(int16_t i = 0;i<23;i++)
    {
      if(strcmp(SOLUTION_STATUSES[i].c_str(),sentence.body[i].c_str()) == 0)
      {
        solution_status = solution_statusId(i);
      }
    }
    if(solution_status>=0){
      msg->position_type = solution_status;
    }
#endif
  valid = valid && ParseDouble(sentence.body[2], msg->latitude);
  valid = valid && ParseDouble(sentence.body[3], msg->longitude);
  valid = valid && ParseDouble(sentence.body[4], msg->altitude);
  valid = valid && ParseFloat(sentence.body[5], msg->undulation);

  valid = valid && ParseFloat(sentence.body[7], msg->latitude_std);
  valid = valid && ParseFloat(sentence.body[8], msg->longitude_std);
  valid = valid && ParseFloat(sentence.body[9], msg->altitude_std);

#if 0
    ROS_DEBUG("ParseAscii1 data:%f,%f,%f,%f,%f,%f,%f",
      msg->latitude,msg->longitude,msg->altitude,
               msg->undulation,msg->latitude_std,msg->longitude_std,msg->altitude_std
               );
#endif

  return msg;
}

uint32_t BestposParser::solution_statusId(uint16_t solution_status) {
  uint32_t solution;
  if (solution_status <= 9) {
    solution = solution_status;
    return solution;
  } else {
    switch (solution_status) {
      case 10:
        solution = 13;
        break;
      case 11:
        solution = 18;
        break;
      case 12:
        solution = 19;
        break;
      case 13:
        solution = 22;
        break;
      default:
        solution = 0;
        break;
    }
    return solution;
  }
}

uint32_t BestposParser::pos_typeId(uint16_t pos_type) {
  uint32_t solution;

  if (pos_type <= 2) {
    solution = pos_type;
  } else if ((pos_type >= 3) && (pos_type <= 5)) {
    solution = pos_type + 1;
  } else if (pos_type == 6) {
    solution = 8;
  } else if ((pos_type >= 7) && (pos_type <= 11)) {
    solution = pos_type + 9;
  } else if ((pos_type >= 12) && (pos_type <= 14)) {
    solution = pos_type + 20;
  } else if ((pos_type >= 15) && (pos_type <= 26)) {
    solution = pos_type + 33;
  } else if ((pos_type >= 27) && (pos_type <= 28)) {
    solution = pos_type + 37;
  } else if ((pos_type >= 29) && (pos_type <= 30)) {
    solution = pos_type + 39;
  } else if ((pos_type >= 31) && (pos_type <= 32)) {
    solution = pos_type + 42;
  }
  return solution;
}
}
#endif
