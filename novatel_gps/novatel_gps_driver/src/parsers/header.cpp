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
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <novatel_gps_driver/parsers/header.h>
#include "novatel_gps_msgs/CommonHeader.h"

#include <ros/ros.h>

uint32_t novatel_gps_driver::HeaderParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::HeaderParser::GetMessageName() const
{
  return "HEADER";
}

novatel_gps_msgs::NovatelMessageHeader novatel_gps_driver::HeaderParser::ParseBinary(
    const novatel_gps_driver::BinaryMessage& bin_msg) throw(ParseException)
{
  // No point in checking whether the port identifier is valid here, because
  // the variable's range is 0-255 and this array has 256 values in it.
  novatel_gps_msgs::NovatelMessageHeader msg;
  msg.port = PORT_IDENTIFIERS[bin_msg.header_.port_address_];
  msg.sequence_num = bin_msg.header_.sequence_;
  msg.percent_idle_time = bin_msg.header_.idle_time_;
  switch (bin_msg.header_.time_status_)
  {
    case 20:
      msg.gps_time_status = "UNKNOWN";
      break;
    case 60:
      msg.gps_time_status = "APPROXIMATE";
      break;
    case 80:
      msg.gps_time_status = "COARSEADJUSTING";
      break;
    case 100:
      msg.gps_time_status = "COARSE";
      break;
    case 120:
      msg.gps_time_status = "COARSESTEERING";
      break;
    case 130:
      msg.gps_time_status = "FREEWHEELING";
      break;
    case 140:
      msg.gps_time_status = "FINEADJUSTING";
      break;
    case 160:
      msg.gps_time_status = "FINE";
      break;
    case 170:
      msg.gps_time_status = "FINEBACKUPSTEERING";
      break;
    case 180:
      msg.gps_time_status = "FINESTEERING";
      break;
    case 200:
      msg.gps_time_status = "SATTIME";
      break;
    default:
    {
      std::stringstream error;
      error << "Unknown GPS time status: " << bin_msg.header_.time_status_;
      throw ParseException(error.str());
    }
  }
  msg.gps_week_num = bin_msg.header_.week_;
  msg.gps_seconds = static_cast<double>(bin_msg.header_.gps_ms_) / 1000.0;
  GetNovatelReceiverStatusMessage(bin_msg.header_.receiver_status_, msg.receiver_status);
  msg.receiver_software_version = bin_msg.header_.receiver_sw_version_;

  return msg;
}

novatel_gps_msgs::NovatelMessageHeader novatel_gps_driver::HeaderParser::ParseAscii(
    const novatel_gps_driver::NovatelSentence& sentence) throw(ParseException)
{
  if (sentence.header.size() != NOVATEL_MESSAGE_HEADER_LENGTH)
  {
    std::stringstream error;
    error <<"Novatel message header size wrong: expected "
          << NOVATEL_MESSAGE_HEADER_LENGTH
          << ", got %zu"<< sentence.header.size();
    throw ParseException(error.str());
  }

  bool valid = true;

  novatel_gps_msgs::NovatelMessageHeader msg;
  msg.message_name = sentence.header[0];

  msg.port = sentence.header[1];
  valid = valid && ParseUInt32(sentence.header[2], msg.sequence_num);
  valid = valid && ParseFloat(sentence.header[3], msg.percent_idle_time);
  msg.gps_time_status = sentence.header[4];
  valid = valid && ParseUInt32(sentence.header[5], msg.gps_week_num);
  valid = valid && ParseDouble(sentence.header[6], msg.gps_seconds);

  uint32_t receiver_status_code = 0;
  valid = valid && ParseUInt32(sentence.header[7], receiver_status_code, 16);
  GetNovatelReceiverStatusMessage(receiver_status_code, msg.receiver_status);

  valid = valid && ParseUInt32(sentence.header[9], msg.receiver_software_version);

  if (!valid)
  {
    throw ParseException("Header was invalid ParseAscii.");
  }
  return msg;
}

/*************
*
*new add
*
*
*************/
#if 1
novatel_gps_msgs::CommonHeader novatel_gps_driver::HeaderParser::ParseBinary1(
    const novatel_gps_driver::BinaryMessage& bin_msg) 
{
  // No point in checking whether the port identifier is valid here, because
  // the variable's range is 0-255 and this array has 256 values in it.
  novatel_gps_msgs::CommonHeader msg;
  //msg.port = PORT_IDENTIFIERS[bin_msg.header_.port_address_];

  msg.msg_type = bin_msg.header_.message_type_;
  msg.port_addr = bin_msg.header_.port_address_;
  msg.length = bin_msg.header_.message_length_;
  msg.sequence = bin_msg.header_.sequence_;
  msg.idle_time = bin_msg.header_.idle_time_;
  msg.time_status = bin_msg.header_.time_status_;
  msg.gps_week = bin_msg.header_.week_;
  msg.gps_mseconds = bin_msg.header_.gps_ms_;
  msg.receiver_status = bin_msg.header_.receiver_status_;
  msg.reserved = bin_msg.header_.reserved_;
  msg.software_version = bin_msg.header_.receiver_sw_version_;
  return msg;

#if 0
  msg.sequence_num = bin_msg.header_.sequence_;
  msg.percent_idle_time = bin_msg.header_.idle_time_;
  switch (bin_msg.header_.time_status_)
  {
    case 20:
      msg.gps_time_status = "UNKNOWN";
      break;
    case 60:
      msg.gps_time_status = "APPROXIMATE";
      break;
    case 80:
      msg.gps_time_status = "COARSEADJUSTING";
      break;
    case 100:
      msg.gps_time_status = "COARSE";
      break;
    case 120:
      msg.gps_time_status = "COARSESTEERING";
      break;
    case 130:
      msg.gps_time_status = "FREEWHEELING";
      break;
    case 140:
      msg.gps_time_status = "FINEADJUSTING";
      break;
    case 160:
      msg.gps_time_status = "FINE";
      break;
    case 170:
      msg.gps_time_status = "FINEBACKUPSTEERING";
      break;
    case 180:
      msg.gps_time_status = "FINESTEERING";
      break;
    case 200:
      msg.gps_time_status = "SATTIME";
      break;
    default:
    {
      std::stringstream error;
      error << "Unknown GPS time status: " << bin_msg.header_.time_status_;
      throw ParseException(error.str());
    }
  }
  msg.gps_week_num = bin_msg.header_.week_;
  msg.gps_seconds = static_cast<double>(bin_msg.header_.gps_ms_) / 1000.0;
  GetNovatelReceiverStatusMessage(bin_msg.header_.receiver_status_, msg.receiver_status);
  msg.receiver_software_version = bin_msg.header_.receiver_sw_version_;
  #endif

}

novatel_gps_msgs::CommonHeader novatel_gps_driver::HeaderParser::ParseAscii1(
    const novatel_gps_driver::NovatelSentence& sentence) 
{
  

  bool valid = true;

  novatel_gps_msgs::CommonHeader msg;

  valid = valid && ParseUInt16(sentence.header[2], msg.sequence);
  valid = valid && ParseUInt8(sentence.header[3], msg.idle_time);
  valid = valid && ParseUInt8(sentence.header[4],msg.time_status);

  valid = valid && ParseUInt16(sentence.header[5], msg.gps_week);
  valid = valid && ParseUInt32(sentence.header[6], msg.gps_mseconds);
  valid = valid && ParseUInt16(sentence.header[9], msg.software_version);

  return msg;
}

#endif


