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

/**
 * \file
 *
 * This nodelet is a driver for Novatel OEM6 and FlexPack6 GPS receivers. It
 * publishes standard ROS gps_common/GPSFix messages, as well as custom
 * NovatelPosition, GPGGA, and GPRMC messages.
 *
 * <b>Topics Subscribed:</b>
 *
 * \e gps_sync <tt>std_msgs/Time<tt> - Timestamped sync pulses
 *    from a DIO module (optional). These are used to improve the accuracy of
 *    the time stamps of the messages published.
 *
 * <b>Topics Published:</b>
 *
 * \e gps <tt>gps_common/GPSFix</tt> - GPS data for navigation
 * \e corrimudata <tt>novatel_gps_message/NovatelCorrectedImuData</tt> - Raw
 *    Novatel IMU data. (only published if `publish_imu_messages` is set `true`)
 * \e gppga <tt>novatel_gps_driver/Gpgga</tt> - Raw GPGGA data for debugging
 * (only published if `publish_nmea_messages` is set `true`) \e gpgsa
 * <tt>novatel_gps_msgs/Gpgsa</tt> - Raw GPGSA data for debugging (only
 *    published if `publish_gpgsa` is set `true`)
 * \e gprmc <tt>novatel_gps_msgs/Gprmc</tt> - Raw GPRMC data for debugging (only
 *    published if `publish_nmea_messages` is set `true`)
 * \e bestpos <tt>novatel_gps_msgs/NovatelPosition</tt> - High fidelity Novatel-
 *    specific position and receiver status data. (only published if
 *    `publish_novatel_positions` is set `true`)
 * \e bestvel <tt>novatel_gps_msgs/NovatelVelocity</tt> - High fidelity Novatel-
 *    specific velocity and receiver status data. (only published if
 *    `publish_novatel_velocity` is set `true`)
 * \e range <tt>novatel_gps_msgs/Range</tt> - Satellite ranging information
 *    (only published if `publish_range_messages` is set `true`)
 * \e time <tt>novatel_gps_msgs/Time</tt> - Novatel-specific time data. (Only
 *    published if `publish_time` is set `true`.)
 * \e trackstat <tt>novatel_gps_msgs/Trackstat</tt> - Novatel-specific trackstat
 *    data at 1 Hz. (Only published if `publish_trackstat` is set `true`.)
 *
 * <b>Services:</b>
 *
 * \e freset <tt>novatel_gps_msgs/NovatelFRESET</tt> - Sends a freset message to
 * the device with the specified target string to reset. By default does FRESET
 * standard
 *
 * <b>Parameters:</b>
 *
 * \e connection_type <tt>str</tt> - "serial", "udp", or "tcp" as appropriate
 *    for the Novatel device connected. ["serial"]
 * \e device <tt>str</tt> - The path to the device, e.g. /dev/ttyUSB0 for
 *    serial connections or "192.168.1.10:3001" for IP.
 *    [""]
 * \e frame_id <tt>str</tt> - The TF frame ID to set in all published message
 *    headers. [""]
 * \e gpgga_gprmc_sync_tol <tt>dbl</tt> - Sync tolarance (seconds) for syncing
 *    GPGGA messages with GPRMC messages. [0.01]
 * \e gpgga_position_sync_tol <tt>dbl</tt> - Sync tolarance (seconds) for
 *    syncing GPGGA messages with BESTPOS messages. [0.01]
 * \e imu_rate <tt>dbl</tt> - Rate at which to publish sensor_msgs/Imu messages.
 *    [100.0]
 * \e imu_sample_rate <tt>dbl</tt> - Rate at which the device internally samples
 *    its IMU. [200.0]
 * \e polling_period <tt>dbl</tt> - The number of seconds in between messages
 *    requested from the GPS. (Does not affect time messages) [0.05]
 * \e publish_diagnostics <tt>bool</tt> - If set true, the driver publishes
 *    ROS diagnostics [true]
 * \e publish_imu_messages <tt>boot</tt> - If set true, the driver publishes
 *    Novatel CorrImuData, InsPva, InsStdev, and sensor_msgs/Imu messages
 * [false] \e publish_gpgsa <tt>bool</tt> - If set true, the driver requests
 * GPGSA messages from the device at 20 Hz and publishes them on `gpgsa` \e
 * publish_nmea_messages <tt>bool</tt> - If set true, the driver publishes GPGGA
 * and GPRMC messages (see Topics Published) [false] \e publish_novatel_messages
 * <tt>bool</tt> - If set true, the driver publishes Novatel Bestpos messages
 * (see Topics Published) [false] \e publish_novatel_velocity <tt>bool</tt> - If
 * set true, the driver publishes Novatel Bestvel messages (see Topics
 * Published) [false] \e publish_range_messages <tt>bool</tt> - If set true, the
 * driver publishes Novatel RANGE messages [false] \e publish_sync_diagnostic
 * <tt>bool</tt> - If true, publish a Sync diagnostic. This is ignored if
 * publish_diagnostics is false. [true] \e publish_time <tt>bool</tt> - If set
 * true, the driver publishes Novatel Time messages (see Topics Published)
 * [false] \e publish_trackstat <tt>bool</tt> - If set true, the driver
 * publishes Novatel Trackstat messages (see Topics Published) [false] \e
 * reconnect_delay_s <tt>bool</t> - If the driver is disconnected from the
 *    device, how long (in seconds) to wait between reconnect attempts. [0.5]
 * \e use_binary_message <tt>bool</tt> - If set true, the driver requests
 *    binary NovAtel messages from the device; if false, it requests ASCII
 *    messages.  [false]
 * \e wait_for_position <tt>bool</tt> - Wait for BESTPOS messages to arrive
 *    before publishing GPSFix messages. [false]
 * \e span_frame_to_ros_frame <tt>bool</tt> - Translate the SPAN coordinate
 *    frame to a ROS coordinate frame using the VEHICLEBODYROTATION and
 *    APPLYVEHICLEBODYROTATION commands. [false]
 */
#include <math.h>
#include <exception>
#include <string>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/tail.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/circular_buffer.hpp>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <gps_common/GPSFix.h>
#include <nodelet/nodelet.h>
#include <novatel_gps_msgs/Gpgga.h>
#include <novatel_gps_msgs/Gprmc.h>
#include <novatel_gps_msgs/NovatelCorrectedImuData.h>
#include <novatel_gps_msgs/NovatelFRESET.h>
#include <novatel_gps_msgs/NovatelMessageHeader.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <novatel_gps_msgs/Range.h>
#include <novatel_gps_msgs/Time.h>

#include <novatel_gps_msgs/BESTPOS1.h>
#include "novatel_gps_msgs/CORRIMUDATA1.h"
#include "novatel_gps_msgs/INSPVA1.h"
#include "novatel_gps_msgs/INSSTDEV1.h"
#include "novatel_gps_msgs//RAWIMU1.h"

#include "novatel_gps_driver/novatel_gps.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include <std_msgs/Time.h>
#include <swri_math_util/math_util.h>
#include <swri_roscpp/parameters.h>
#include <swri_roscpp/publisher.h>
#include <swri_roscpp/subscriber.h>

/*************new add by cgl***************/
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
//#include <geodesy/utm.h>
#include <geometry_msgs/Point.h>
#include <gps_common/conversions.h>
#include <tf/tf.h>
#include <map>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "common_param.h"

#include <ros/package.h>
namespace stats = boost::accumulators;

namespace novatel_gps_driver {
class NovatelGpsNodelet : public nodelet::Nodelet {
 public:
  NovatelGpsNodelet()
      : device_("/dev/ttyUSB0"),
        connection_type_("serial"),
        polling_period_(0.05),
        publish_gpgsa_(false),
        publish_gpgsv_(false),
        imu_rate_(100.0),
        imu_sample_rate_(125.0),
        publish_imu_messages_(false),
        publish_novatel_positions_(true),
        publish_novatel_velocity_(false),
        publish_nmea_messages_(false),
        publish_range_messages_(false),
        publish_time_messages_(false),
        publish_trackstat_(false),
        publish_diagnostics_(true),
        publish_sync_diagnostic_(false),
        reconnect_delay_s_(0.5),
        use_binary_messages_(true),
        span_frame_to_ros_frame_(false),
        connection_(NovatelGps::SERIAL),
        last_sync_(ros::TIME_MIN),
        rolling_offset_(stats::tag::rolling_window::window_size = 10),
        expected_rate_(2),
        device_timeouts_(0),
        device_interrupts_(0),
        device_errors_(0),
        gps_parse_failures_(0),
        gps_insufficient_data_warnings_(0),
        publish_rate_warnings_(0),
        measurement_count_(0),
        last_published_(ros::TIME_MIN),
        imu_frame_id_(""),
        frame_id_("") {
    odom_init = false;
#if 0
      ros::NodeHandle nh;

      nh.param("publish_tf", publish_tf, false);
      nh.param("odom_frame", odom_frame, "odom_combined");
      nh.param("base_frame", base_frame, "base_link");
      nh.param("zero_start", zero_start, false);
      nh.param("rate", imu_rate, 100);
#endif
    double IMU_ORIENT_COVAR[9] = {1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3};
    orientation_covariance.insert(orientation_covariance.begin(),
                                  IMU_ORIENT_COVAR, IMU_ORIENT_COVAR + 9);

    orientation.insert(orientation.begin(), 4, 0);

    double NAVSAT_COVAR[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    position_covariance.insert(position_covariance.begin(), NAVSAT_COVAR,
                               NAVSAT_COVAR + 9);

    double IMU_VEL_COVAR[9] = {1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3};
    linear_velocity_covariance.insert(linear_velocity_covariance.begin(),
                                      IMU_VEL_COVAR, IMU_VEL_COVAR + 9);

  }

  ~NovatelGpsNodelet() { gps_.Disconnect(); }

  /**
   * Init method reads parameters and sets up publishers and subscribers.
   * It does not connect to the device.
   */
  void onInit() {
    ros::NodeHandle& node = getNodeHandle();
    ros::NodeHandle& priv = getPrivateNodeHandle();

    swri::param(priv, "device", device_, device_);
    swri::param(priv, "imu_rate", imu_rate_, imu_rate_);
    swri::param(priv, "imu_sample_rate", imu_sample_rate_, imu_sample_rate_);
    swri::param(priv, "publish_gpgsa", publish_gpgsa_, publish_gpgsa_);
    swri::param(priv, "publish_gpgsv", publish_gpgsv_, publish_gpgsv_);
    swri::param(priv, "publish_imu_messages", publish_imu_messages_,
                publish_imu_messages_);
    swri::param(priv, "publish_novatel_positions", publish_novatel_positions_,
                publish_novatel_positions_);
    swri::param(priv, "publish_novatel_velocity", publish_novatel_velocity_,
                publish_novatel_velocity_);
    swri::param(priv, "publish_nmea_messages", publish_nmea_messages_,
                publish_nmea_messages_);
    swri::param(priv, "publish_range_messages", publish_range_messages_,
                publish_range_messages_);
    swri::param(priv, "publish_time_messages", publish_time_messages_,
                publish_time_messages_);
    swri::param(priv, "publish_trackstat", publish_trackstat_,
                publish_trackstat_);
    swri::param(priv, "publish_diagnostics", publish_diagnostics_,
                publish_diagnostics_);
    swri::param(priv, "publish_sync_diagnostic", publish_sync_diagnostic_,
                publish_sync_diagnostic_);
    swri::param(priv, "polling_period", polling_period_, polling_period_);
    swri::param(priv, "reconnect_delay_s", reconnect_delay_s_,
                reconnect_delay_s_);
    swri::param(priv, "use_binary_messages", use_binary_messages_,
                use_binary_messages_);
    swri::param(priv, "span_frame_to_ros_frame", span_frame_to_ros_frame_,
                span_frame_to_ros_frame_);

    swri::param(priv, "connection_type", connection_type_, connection_type_);
    connection_ = NovatelGps::ParseConnection(connection_type_);

    swri::param(priv, "imu_frame_id", imu_frame_id_, std::string(""));
    swri::param(priv, "frame_id", frame_id_, std::string("/gps"));

    // set NovatelGps parameters
    swri::param(priv, "gpgga_gprmc_sync_tol", gps_.gpgga_gprmc_sync_tol_, 0.01);
    swri::param(priv, "gpgga_position_sync_tol", gps_.gpgga_position_sync_tol_,
                0.01);
    swri::param(priv, "wait_for_position", gps_.wait_for_position_, false);

    // new add
    swri::param(priv, "publish_tf", publish_tf, false);
    swri::param(priv, "odom_frame", odom_frame, "odom_combined");
    swri::param(priv, "base_frame", base_frame, "base_link");
    swri::param(priv, "zero_start", zero_start, false);
    swri::param(priv, "rate", imu_rate, 100);

    // Reset Service
    reset_service_ =
        priv.advertiseService("freset", &NovatelGpsNodelet::resetService, this);

    sync_sub_ = swri::Subscriber(node, "gps_sync", 100,
                                 &NovatelGpsNodelet::SyncCallback, this);

    std::string gps_topic = node.resolveName("gps");
    gps_pub_ = swri::advertise<gps_common::GPSFix>(node, gps_topic, 100);
    fix_pub_ = swri::advertise<sensor_msgs::NavSatFix>(node, "fix", 100);

    if (publish_nmea_messages_) {
      gpgga_pub_ = swri::advertise<novatel_gps_msgs::Gpgga>(node, "gpgga", 100);
      gprmc_pub_ = swri::advertise<novatel_gps_msgs::Gprmc>(node, "gprmc", 100);
    }

    if (publish_gpgsa_) {
      gpgsa_pub_ = swri::advertise<novatel_gps_msgs::Gpgsa>(node, "gpgsa", 100);
    }

    if (publish_imu_messages_) {
      imu_pub_ = swri::advertise<sensor_msgs::Imu>(node, "imu", 100);
      novatel_imu_pub_ =
          swri::advertise<novatel_gps_msgs::NovatelCorrectedImuData>(
              node, "corrimudata", 100);
      insstdev_pub_ =
          swri::advertise<novatel_gps_msgs::Insstdev>(node, "insstdev", 100);
      inspva_pub_ =
          swri::advertise<novatel_gps_msgs::Inspva>(node, "inspva", 100);
      inscov_pub_ =
          swri::advertise<novatel_gps_msgs::Inscov>(node, "inscov", 100);
    }

    if (publish_gpgsv_) {
      gpgsv_pub_ = swri::advertise<novatel_gps_msgs::Gpgsv>(node, "gpgsv", 100);
    }

    if (publish_novatel_positions_) {
      novatel_position_pub_ =
          swri::advertise<novatel_gps_msgs::NovatelPosition>(node, "bestpos",
                                                             100);
    }

    if (publish_novatel_velocity_) {
      novatel_velocity_pub_ =
          swri::advertise<novatel_gps_msgs::NovatelVelocity>(node, "bestvel",
                                                             100);
    }

    if (publish_range_messages_) {
      range_pub_ = swri::advertise<novatel_gps_msgs::Range>(node, "range", 100);
    }

    if (publish_time_messages_) {
      time_pub_ = swri::advertise<novatel_gps_msgs::Time>(node, "time", 100);
    }

    if (publish_trackstat_) {
      trackstat_pub_ =
          swri::advertise<novatel_gps_msgs::Trackstat>(node, "trackstat", 100);
    }

    add_new_publish_adver();

    hw_id_ = "Novatel GPS";
    if (publish_diagnostics_) {
      diagnostic_updater_.setHardwareID(hw_id_);
      diagnostic_updater_.add("Connection", this,
                              &NovatelGpsNodelet::DeviceDiagnostic);
      diagnostic_updater_.add("Hardware", this,
                              &NovatelGpsNodelet::GpsDiagnostic);
      diagnostic_updater_.add("Data", this, &NovatelGpsNodelet::DataDiagnostic);
      diagnostic_updater_.add("Rate", this, &NovatelGpsNodelet::RateDiagnostic);
      diagnostic_updater_.add("GPS Fix", this,
                              &NovatelGpsNodelet::FixDiagnostic);
      if (publish_sync_diagnostic_) {
        diagnostic_updater_.add("Sync", this,
                                &NovatelGpsNodelet::SyncDiagnostic);
      }
    }

    gps_.ApplyVehicleBodyRotation(span_frame_to_ros_frame_);

    thread_ = boost::thread(&NovatelGpsNodelet::Spin, this);
    NODELET_INFO("%s initialized", hw_id_.c_str());
  }

  void SyncCallback(const std_msgs::TimeConstPtr& sync) {
    boost::unique_lock<boost::mutex> lock(mutex_);
    sync_times_.push_back(sync->data);
  }

  /**
   * Main spin loop connects to device, then reads data from it and publishes
   * messages.
   */
  void Spin() {
    std::string format_suffix;
    if (use_binary_messages_) {
      // NovAtel logs in binary format always end with "b", while ones in
      // ASCII format always end in "a".
      format_suffix = "b";
    } else {
      format_suffix = "a";
    }

    NovatelMessageOpts opts;
    opts["gpgga"] = polling_period_;
    opts["gprmc"] = polling_period_;
    opts["bestpos" + format_suffix] = polling_period_;  // Best position
    opts["time" + format_suffix] = 1.0;                 // Time

    double period = 1.0 / imu_rate_;
    opts["corrimudata" + format_suffix] = period;  // 0.02
    opts["inscov" + format_suffix] = 1.0;
    opts["inspva" + format_suffix] = period;
    opts["insstdev" + format_suffix] = 1.0;

    if (publish_gpgsa_) {
      opts["gpgsa"] = polling_period_;
    }
    if (publish_gpgsv_) {
      opts["gpgsv"] = 1.0;
    }
    if (publish_imu_messages_) {
      if (!use_binary_messages_) {
        NODELET_WARN(
            "Using the ASCII message format with CORRIMUDATA logs is not "
            "recommended.  "
            "A serial link will not be able to keep up with the data rate.");
      }

      // Only configure the imu rate if we set the param
      if (imu_sample_rate_ > 0) {
        gps_.SetImuRate(imu_sample_rate_, true);
      }
    }
    if (publish_novatel_velocity_) {
      opts["bestvel" + format_suffix] = polling_period_;  // Best velocity
    }
    if (publish_range_messages_) {
      opts["range" + format_suffix] = 1.0;  // Range. 1 msg/sec is max rate
    }
    if (publish_trackstat_) {
      opts["trackstat" + format_suffix] = 1.0;  // Trackstat
    }
    while (ros::ok()) {
#if 0
        TEST_data();

#else
      if (gps_.Connect(device_, connection_, opts)) {
        // Connected to device. Begin reading/processing data
        NODELET_INFO("%s connected to device", hw_id_.c_str());
        while (gps_.IsConnected() && ros::ok()) {
          // Read data from the device and publish any received messages
          CheckDeviceForData();

          Check_new_DeviceForData();

          // Poke the diagnostic updater. It will only fire diagnostics if
          // its internal timer (1 Hz) has elapsed. Otherwise, this is a
          // noop
          if (publish_diagnostics_) {
            diagnostic_updater_.update();
          }

          // Spin once to let the ROS callbacks fire
          ros::spinOnce();
          // Sleep for a microsecond to prevent CPU hogging
          boost::this_thread::sleep(boost::posix_time::microseconds(1));
        }     // While (gps_.IsConnected() && ros::ok()) (inner loop to process
              // data from device)
      } else  // Could not connect to the device
      {
        NODELET_ERROR_THROTTLE(1, "Error connecting to device <%s:%s>: %s",
                               connection_type_.c_str(), device_.c_str(),
                               gps_.ErrorMsg().c_str());
        device_errors_++;
        error_msg_ = gps_.ErrorMsg();
      }
#endif
      if (ros::ok()) {
        // If ROS is still OK but we got disconnected, we're going to try
        // to reconnect, but wait just a bit so we don't spam the device.
        ros::Duration(reconnect_delay_s_).sleep();
      }

      // Poke the diagnostic updater. It will only fire diagnostics if
      // its internal timer (1 Hz) has elapsed. Otherwise, this is a
      // noop
      if (publish_diagnostics_) {
        diagnostic_updater_.update();
      }

      // Spin once to let the ROS callbacks fire
      ros::spinOnce();
      // Sleep for a microsecond to prevent CPU hogging
      boost::this_thread::sleep(boost::posix_time::microseconds(1));

      if (connection_ == NovatelGps::PCAP) {
        // If we're playing a PCAP file, we only want to play it once.
        ros::shutdown();
      }
    }  // While (ros::ok) (outer loop to reconnect to device)

    gps_.Disconnect();
    NODELET_INFO("%s disconnected and shut down", hw_id_.c_str());
  }

 private:
  // Parameters
  /// The device identifier e.g. /dev/ttyUSB0
  std::string device_;
  /// The connection type, ("serial", "tcp", or "udp")
  std::string connection_type_;
  double polling_period_;
  bool publish_gpgsa_;
  bool publish_gpgsv_;
  /// The rate at which IMU measurements will be published, in Hz
  double imu_rate_;
  /// How frequently the device samples the IMU, in Hz
  double imu_sample_rate_;
  bool span_frame_to_ros_frame_;
  bool publish_imu_messages_;
  bool publish_novatel_positions_;
  bool publish_novatel_velocity_;
  bool publish_nmea_messages_;
  bool publish_range_messages_;
  bool publish_time_messages_;
  bool publish_trackstat_;
  bool publish_diagnostics_;
  bool publish_sync_diagnostic_;
  double reconnect_delay_s_;
  bool use_binary_messages_;

  ros::Publisher fix_pub_;
  ros::Publisher gps_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher inscov_pub_;
  ros::Publisher inspva_pub_;
  ros::Publisher insstdev_pub_;
  ros::Publisher novatel_imu_pub_;
  ros::Publisher novatel_position_pub_;
  ros::Publisher novatel_velocity_pub_;
  ros::Publisher gpgga_pub_;
  ros::Publisher gpgsv_pub_;
  ros::Publisher gpgsa_pub_;
  ros::Publisher gprmc_pub_;
  ros::Publisher range_pub_;
  ros::Publisher time_pub_;
  ros::Publisher trackstat_pub_;

  /*************new add by clg******************/
  ros::Publisher pub_imu;
  ros::Publisher pub_raw_imu;
  ros::Publisher pub_odom;
  ros::Publisher pub_origin;
  ros::Publisher pub_navsatfix;
  bool odom_init;
  bool zero_start;
  bool publish_tf;
  int imu_rate;
  std::string odom_frame;
  std::string base_frame;
  geometry_msgs::Pose origin;

  std::vector<double> orientation_covariance;

  std::vector<double> orientation;

  std::vector<double> position_covariance;
  std::vector<double> linear_velocity_covariance;
  /*******************************************/

  ros::ServiceServer reset_service_;

  NovatelGps::ConnectionType connection_;
  NovatelGps gps_;

  boost::thread thread_;
  boost::mutex mutex_;

  /// Subscriber to listen for sync times from a DIO
  swri::Subscriber sync_sub_;
  ros::Time last_sync_;
  /// Buffer of sync message time stamps
  boost::circular_buffer<ros::Time> sync_times_;
  /// Buffer of gps message time stamps
  boost::circular_buffer<ros::Time> msg_times_;

  /// Stats on time offset
  stats::accumulator_set<float,
                         stats::stats<stats::tag::max, stats::tag::min,
                                      stats::tag::mean, stats::tag::variance> >
      offset_stats_;
  /// Rolling mean of time offset
  stats::accumulator_set<float, stats::stats<stats::tag::rolling_mean> >
      rolling_offset_;

  // ROS diagnostics
  std::string error_msg_;
  diagnostic_updater::Updater diagnostic_updater_;
  std::string hw_id_;
  double expected_rate_;
  int32_t device_timeouts_;
  int32_t device_interrupts_;
  int32_t device_errors_;
  int32_t gps_parse_failures_;
  int32_t gps_insufficient_data_warnings_;
  int32_t publish_rate_warnings_;
  int32_t measurement_count_;
  ros::Time last_published_;
  novatel_gps_msgs::NovatelPositionPtr last_novatel_position_;

  std::string imu_frame_id_;
  std::string frame_id_;

  uint rtk_temp_status;
  std::string temp_status;
  std::string ins_status;

  /**
   * @brief Service request to reset the gps through FRESET
   */
  bool resetService(novatel_gps_msgs::NovatelFRESET::Request& req,
                    novatel_gps_msgs::NovatelFRESET::Response& res) {
    if (gps_.IsConnected() == false) {
      res.success = false;
    }

    // Formulate the reset command and send it to the device
    std::string command = "FRESET ";
    command += req.target.length() ? "STANDARD" : req.target;
    command += "\r\n";
    gps_.Write(command);

    if (req.target.length() == 0) {
      ROS_WARN(
          "No FRESET target specified. Doing FRESET STANDARD. This may be "
          "undesired behavior.");
    }

    res.success = true;
    return true;
  }

  /**
   * @brief Reads data from the device and publishes any parsed messages.
   *
   * Note that when reading from the device, this will block until data
   * is available.
   */
  void CheckDeviceForData() {
    std::vector<novatel_gps_msgs::NovatelPositionPtr> position_msgs;

    std::vector<gps_common::GPSFixPtr> fix_msgs;
    std::vector<novatel_gps_msgs::GpggaPtr> gpgga_msgs;
    std::vector<novatel_gps_msgs::GprmcPtr> gprmc_msgs;

    // This call appears to block if the serial device is disconnected
    NovatelGps::ReadResult result = gps_.ProcessData();
    if (result == NovatelGps::READ_ERROR) {
      NODELET_ERROR_THROTTLE(1, "Error reading from device <%s:%s>: %s",
                             connection_type_.c_str(), device_.c_str(),
                             gps_.ErrorMsg().c_str());
      device_errors_++;
    } else if (result == NovatelGps::READ_TIMEOUT) {
      device_timeouts_++;
    } else if (result == NovatelGps::READ_INTERRUPTED) {
      // If we are interrupted by a signal, ROS is probably
      // quitting, but we'll wait for ROS to tell us to quit.
      device_interrupts_++;
    } else if (result == NovatelGps::READ_PARSE_FAILED) {
      NODELET_ERROR("Error reading from device <%s:%s>: %s",
                    connection_type_.c_str(), device_.c_str(),
                    gps_.ErrorMsg().c_str());
      gps_parse_failures_++;
    } else if (result == NovatelGps::READ_INSUFFICIENT_DATA) {
      gps_insufficient_data_warnings_++;
    }

    // Read messages from the driver into the local message lists
    gps_.GetGpggaMessages(gpgga_msgs);
    gps_.GetGprmcMessages(gprmc_msgs);
    gps_.GetNovatelPositions(position_msgs);

    gps_.GetFixMessages(fix_msgs);

    // Increment the measurement count by the number of messages we just
    // read
    measurement_count_ += gpgga_msgs.size();

    // If there are new position messages, store the most recent
    if (!position_msgs.empty()) {
      last_novatel_position_ = position_msgs.back();
    }

    // Find all the gppga messages that are within 20 ms of whole
    // numbered UTC seconds and push their stamps onto the msg_times
    // buffer
    for (const auto& msg : gpgga_msgs) {
      if (msg->utc_seconds != 0) {
        int64_t second =
            static_cast<int64_t>(swri_math_util::Round(msg->utc_seconds));
        double difference = std::fabs(msg->utc_seconds - second);

        if (difference < 0.02) {
          msg_times_.push_back(msg->header.stamp);
        }
      }
    }

    // If timesync messages are available, CalculateTimeSync will
    // update a stat accumulator of the offset of the TimeSync message
    // stamp from the GPS message stamp
    CalculateTimeSync();

    // If TimeSync messages are available, CalculateTimeSync keeps
    // an acculumator of their offset, which is used to
    // calculate a rolling mean of the offset to apply to all messages
    ros::Duration sync_offset(0);  // If no TimeSyncs, assume 0 offset
    if (last_sync_ != ros::TIME_MIN) {
      sync_offset = ros::Duration(stats::rolling_mean(rolling_offset_));
    }
    NODELET_DEBUG_STREAM("GPS TimeSync offset is " << sync_offset);

    if (publish_nmea_messages_) {
      for (const auto& msg : gpgga_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = frame_id_;
        gpgga_pub_.publish(msg);
      }

      for (const auto& msg : gprmc_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = frame_id_;
        gprmc_pub_.publish(msg);
      }
    }

    if (publish_gpgsa_) {
      std::vector<novatel_gps_msgs::GpgsaPtr> gpgsa_msgs;
      gps_.GetGpgsaMessages(gpgsa_msgs);
      for (const auto& msg : gpgsa_msgs) {
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = frame_id_;
        gpgsa_pub_.publish(msg);
      }
    }

    if (publish_gpgsv_) {
      std::vector<novatel_gps_msgs::GpgsvPtr> gpgsv_msgs;
      gps_.GetGpgsvMessages(gpgsv_msgs);
      for (const auto& msg : gpgsv_msgs) {
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = frame_id_;
        gpgsv_pub_.publish(msg);
      }
    }

    if (publish_novatel_positions_) {
      for (const auto& msg : position_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = frame_id_;
        novatel_position_pub_.publish(msg);
      }
    }

    if (publish_novatel_velocity_) {
      std::vector<novatel_gps_msgs::NovatelVelocityPtr> velocity_msgs;
      gps_.GetNovatelVelocities(velocity_msgs);
      for (const auto& msg : velocity_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = frame_id_;
        novatel_velocity_pub_.publish(msg);
      }
    }
    if (publish_time_messages_) {
      std::vector<novatel_gps_msgs::TimePtr> time_msgs;
      gps_.GetTimeMessages(time_msgs);
      for (const auto& msg : time_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = frame_id_;
        time_pub_.publish(msg);
      }
    }
    if (publish_range_messages_) {
      std::vector<novatel_gps_msgs::RangePtr> range_msgs;
      gps_.GetRangeMessages(range_msgs);
      for (const auto& msg : range_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = frame_id_;
        range_pub_.publish(msg);
      }
    }
    if (publish_trackstat_) {
      std::vector<novatel_gps_msgs::TrackstatPtr> trackstat_msgs;
      gps_.GetTrackstatMessages(trackstat_msgs);
      for (const auto& msg : trackstat_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = frame_id_;
        trackstat_pub_.publish(msg);
      }
    }
    if (publish_imu_messages_) {
      std::vector<novatel_gps_msgs::NovatelCorrectedImuDataPtr>
          novatel_imu_msgs;
      gps_.GetNovatelCorrectedImuData(novatel_imu_msgs);
      for (const auto& msg : novatel_imu_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = imu_frame_id_;
        novatel_imu_pub_.publish(msg);
      }

      std::vector<sensor_msgs::ImuPtr> imu_msgs;
      gps_.GetImuMessages(imu_msgs);
      for (const auto& msg : imu_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = imu_frame_id_;
        imu_pub_.publish(msg);
      }

      std::vector<novatel_gps_msgs::InscovPtr> inscov_msgs;
      gps_.GetInscovMessages(inscov_msgs);
      for (const auto& msg : inscov_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = imu_frame_id_;
        inscov_pub_.publish(msg);
      }

      std::vector<novatel_gps_msgs::InspvaPtr> inspva_msgs;
      gps_.GetInspvaMessages(inspva_msgs);
      for (const auto& msg : inspva_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = imu_frame_id_;
        inspva_pub_.publish(msg);

        // creat_odom_package(msg);
      }

      std::vector<novatel_gps_msgs::InsstdevPtr> insstdev_msgs;
      gps_.GetInsstdevMessages(insstdev_msgs);
      for (const auto& msg : insstdev_msgs) {
        msg->header.stamp += sync_offset;
        msg->header.frame_id = imu_frame_id_;
        insstdev_pub_.publish(msg);
      }
    }

    for (const auto& msg : fix_msgs) {
      msg->header.stamp += sync_offset;
      msg->header.frame_id = frame_id_;
      gps_pub_.publish(msg);

      if (fix_pub_.getNumSubscribers() > 0) {
        sensor_msgs::NavSatFixPtr fix_msg = ConvertGpsFixToNavSatFix(msg);

        fix_pub_.publish(fix_msg);
      }

      // If the time between GPS message stamps is greater than 1.5
      // times the expected publish rate, increment the
      // publish_rate_warnings_ counter, which is used in diagnostics
      if (last_published_ != ros::TIME_MIN &&
          (msg->header.stamp - last_published_).toSec() >
              1.5 * (1.0 / expected_rate_)) {
        publish_rate_warnings_++;
      }

      last_published_ = msg->header.stamp;
    }
  }

  void Check_new_DeviceForData() {
    std::vector<novatel_gps_msgs::BESTPOS1Ptr> position_msgs1;
    gps_.GetNovatelPositions1(position_msgs1);
    for (std::vector<novatel_gps_msgs::BESTPOS1Ptr>::iterator iter =
             position_msgs1.begin();
         iter != position_msgs1.end(); iter++) {
      creat_navsatfix(*iter);
    }

//      std::vector<novatel_gps_msgs::INSPVA1Ptr> position_msgs1;
//      gps_.GetInspvaMessages1(position_msgs1);
//      for (std::vector<novatel_gps_msgs::INSPVA1Ptr>::iterator iter =
//               position_msgs1.begin();
//           iter != position_msgs1.end(); iter++) {
//        creat_navsatfix(*iter);
//      }

    std::vector<novatel_gps_msgs::CORRIMUDATA1Ptr> novatel_imu_msgs1;
    gps_.GetNovatelCorrectedImuData1(novatel_imu_msgs1);
    for (std::vector<novatel_gps_msgs::CORRIMUDATA1Ptr>::iterator iter =
             novatel_imu_msgs1.begin();
         iter != novatel_imu_msgs1.end(); iter++) {
      creat_imu_pac(*iter);
    }

    std::vector<novatel_gps_msgs::RAWIMU1Ptr> novatel_raw_imu_msgs1;
    gps_.GetNovatelRawImu1(novatel_raw_imu_msgs1);
    for(std::vector<novatel_gps_msgs::RAWIMU1Ptr>::iterator iter = novatel_raw_imu_msgs1.begin(); iter != novatel_raw_imu_msgs1.end(); iter++)
    {
        create_raw_imu_msgs(*iter);
    }

    std::vector<novatel_gps_msgs::INSPVA1Ptr> inspva_msgs1;
    gps_.GetInspvaMessages1(inspva_msgs1);
    for (std::vector<novatel_gps_msgs::INSPVA1Ptr>::iterator iter =
             inspva_msgs1.begin();
         iter != inspva_msgs1.end(); iter++) {
      creat_odom_pose_msg(*iter);
    }

    std::vector<novatel_gps_msgs::INSSTDEV1Ptr> insstdev_msgs1;
    gps_.GetInsstdevMessages1(insstdev_msgs1);

    for (std::vector<novatel_gps_msgs::INSSTDEV1Ptr>::iterator iter =
             insstdev_msgs1.begin();
         iter != insstdev_msgs1.end(); iter++) {
      insstdev_handler(*iter);
    }
  }

  sensor_msgs::NavSatFixPtr ConvertGpsFixToNavSatFix(
      const gps_common::GPSFixPtr& msg) {
    sensor_msgs::NavSatFixPtr fix_msg =
        boost::make_shared<sensor_msgs::NavSatFix>();
    fix_msg->header = msg->header;
    fix_msg->latitude = msg->latitude;
    fix_msg->longitude = msg->longitude;
    fix_msg->altitude = msg->altitude;
    fix_msg->position_covariance = msg->position_covariance;
    switch (msg->status.status) {
      case gps_common::GPSStatus::STATUS_NO_FIX:
        fix_msg->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        break;
      case gps_common::GPSStatus::STATUS_FIX:
        fix_msg->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        break;
      case gps_common::GPSStatus::STATUS_SBAS_FIX:
        fix_msg->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        break;
      case gps_common::GPSStatus::STATUS_GBAS_FIX:
        fix_msg->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        break;
      case gps_common::GPSStatus::STATUS_DGPS_FIX:
      case gps_common::GPSStatus::STATUS_WAAS_FIX:
      default:
        ROS_WARN_ONCE("Unsupported fix status: %d", msg->status.status);
        fix_msg->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        break;
    }
    switch (msg->position_covariance_type) {
      case gps_common::GPSFix::COVARIANCE_TYPE_KNOWN:
        fix_msg->position_covariance_type =
            sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
        break;
      case gps_common::GPSFix::COVARIANCE_TYPE_APPROXIMATED:
        fix_msg->position_covariance_type =
            sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        break;
      case gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN:
        fix_msg->position_covariance_type =
            sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        break;
      case gps_common::GPSFix::COVARIANCE_TYPE_UNKNOWN:
        fix_msg->position_covariance_type =
            sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        break;
      default:
        ROS_WARN_ONCE("Unsupported covariance type: %d",
                      msg->position_covariance_type);
        fix_msg->position_covariance_type =
            sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        break;
    }
    // TODO Figure out how to set this based on info in a GPSFix
    fix_msg->status.service = 0;

    return fix_msg;
  }

  /**
   * Updates the time sync offsets by matching up timesync messages to gps
   * messages and calculating the time offset between them.
   */
  void CalculateTimeSync() {
    boost::unique_lock<boost::mutex> lock(mutex_);
    int32_t synced_i = -1;  /// Index of last synced timesync msg
    int32_t synced_j = -1;  /// Index of last synced gps msg
    // Loop over sync times buffer
    for (int32_t i = 0; i < sync_times_.size(); i++) {
      // Loop over message times buffer
      for (int32_t j = synced_j + 1; j < msg_times_.size(); j++) {
        // Offset is the difference between the sync time and message time
        double offset = (sync_times_[i] - msg_times_[j]).toSec();
        if (std::fabs(offset) < 0.49) {
          // If the offset is less than 0.49 sec, the messages match
          synced_i = static_cast<int32_t>(i);
          synced_j = static_cast<int32_t>(j);
          // Add the offset to the stats accumulators
          offset_stats_(offset);
          rolling_offset_(offset);
          // Update the last sync
          last_sync_ = sync_times_[i];
          // Break out of the inner loop and continue looping through sync times
          break;
        }
      }
    }

    // Remove all the timesync messages that have been matched from the queue
    for (int i = 0; i <= synced_i && !sync_times_.empty(); i++) {
      sync_times_.pop_front();
    }

    // Remove all the gps messages that have been matched from the queue
    for (int j = 0; j <= synced_j && !msg_times_.empty(); j++) {
      msg_times_.pop_front();
    }
  }

  void FixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.clear();
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

    if (!last_novatel_position_) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No Status");
      NODELET_WARN("No GPS status data.");
      return;
    }

    status.add("Solution Status", last_novatel_position_->solution_status);
    status.add("Position Type", last_novatel_position_->position_type);
    status.add("Solution Age", last_novatel_position_->solution_age);
    status.add(
        "Satellites Tracked",
        static_cast<int>(last_novatel_position_->num_satellites_tracked));
    status.add("Satellites Used",
               static_cast<int>(
                   last_novatel_position_->num_satellites_used_in_solution));
    status.add(
        "Software Version",
        last_novatel_position_->novatel_msg_header.receiver_software_version);

    const novatel_gps_msgs::NovatelReceiverStatus& rcvr_status =
        last_novatel_position_->novatel_msg_header.receiver_status;
    status.add("Status Code", rcvr_status.original_status_code);

    if (last_novatel_position_->novatel_msg_header.receiver_status
            .original_status_code != 0) {
      uint8_t level = diagnostic_msgs::DiagnosticStatus::WARN;
      std::string msg = "Status Warning";
      // If the antenna is disconnected/broken, this is an error
      if (rcvr_status.antenna_is_open || rcvr_status.antenna_is_shorted ||
          !rcvr_status.antenna_powered) {
        msg += " Antenna Problem";
        level = diagnostic_msgs::DiagnosticStatus::ERROR;
      }
      status.add("Error Flag", rcvr_status.error_flag ? "true" : "false");
      status.add("Temperature Flag",
                 rcvr_status.temperature_flag ? "true" : "false");
      status.add("Voltage Flag",
                 rcvr_status.voltage_supply_flag ? "true" : "false");
      status.add("Antenna Not Powered",
                 rcvr_status.antenna_powered ? "false" : "true");
      status.add("Antenna Open",
                 rcvr_status.antenna_is_open ? "true" : "false");
      status.add("Antenna Shorted",
                 rcvr_status.antenna_is_shorted ? "true" : "false");
      status.add("CPU Overloaded",
                 rcvr_status.cpu_overload_flag ? "true" : "false");
      status.add("COM1 Buffer Overrun",
                 rcvr_status.com1_buffer_overrun ? "true" : "false");
      status.add("COM2 Buffer Overrun",
                 rcvr_status.com2_buffer_overrun ? "true" : "false");
      status.add("COM3 Buffer Overrun",
                 rcvr_status.com3_buffer_overrun ? "true" : "false");
      status.add("USB Buffer Overrun",
                 rcvr_status.usb_buffer_overrun ? "true" : "false");
      status.add("RF1 AGC Flag", rcvr_status.rf1_agc_flag ? "true" : "false");
      status.add("RF2 AGC Flag", rcvr_status.rf2_agc_flag ? "true" : "false");
      status.add("Almanac Flag", rcvr_status.almanac_flag ? "true" : "false");
      status.add("Position Solution Flag",
                 rcvr_status.position_solution_flag ? "true" : "false");
      status.add("Position Fixed Flag",
                 rcvr_status.position_fixed_flag ? "true" : "false");
      status.add("Clock Steering Status",
                 rcvr_status.clock_steering_status_enabled ? "true" : "false");
      status.add("Clock Model Flag",
                 rcvr_status.clock_model_flag ? "true" : "false");
      status.add("OEMV External Oscillator Flag",
                 rcvr_status.oemv_external_oscillator_flag ? "true" : "false");
      status.add("Software Resource Flag",
                 rcvr_status.software_resource_flag ? "true" : "false");
      status.add("Auxiliary1 Flag",
                 rcvr_status.aux1_status_event_flag ? "true" : "false");
      status.add("Auxiliary2 Flag",
                 rcvr_status.aux2_status_event_flag ? "true" : "false");
      status.add("Auxiliary3 Flag",
                 rcvr_status.aux3_status_event_flag ? "true" : "false");
      //NODELET_WARN("Novatel status code: %d", rcvr_status.original_status_code);
      status.summary(level, msg);
    }
  }

  void SyncDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

    if (last_sync_ == ros::TIME_MIN) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No Sync");
      return;
    } else if (last_sync_ < ros::Time::now() - ros::Duration(10)) {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Sync Stale");
      NODELET_ERROR("GPS time synchronization is stale.");
    }

    status.add("Last Sync", last_sync_);
    status.add("Mean Offset", stats::mean(offset_stats_));
    status.add("Mean Offset (rolling)", stats::rolling_mean(rolling_offset_));
    status.add("Offset Variance", stats::variance(offset_stats_));
    status.add("Min Offset", stats::min(offset_stats_));
    status.add("Max Offset", stats::max(offset_stats_));
  }

  void DeviceDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

    if (device_errors_ > 0) {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Device Errors");
    } else if (device_interrupts_ > 0) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Device Interrupts");
      NODELET_WARN("device interrupts detected <%s:%s>: %d",
                   connection_type_.c_str(), device_.c_str(),
                   device_interrupts_);
    } else if (device_timeouts_) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Device Timeouts");
      NODELET_WARN("device timeouts detected <%s:%s>: %d",
                   connection_type_.c_str(), device_.c_str(), device_timeouts_);
    }

    status.add("Errors", device_errors_);
    status.add("Interrupts", device_interrupts_);
    status.add("Timeouts", device_timeouts_);

    device_timeouts_ = 0;
    device_interrupts_ = 0;
    device_errors_ = 0;
  }

  void GpsDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

    if (gps_parse_failures_ > 0) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Parse Failures");
      NODELET_WARN("gps parse failures detected <%s>: %d", hw_id_.c_str(),
                   gps_parse_failures_);
    }

    status.add("Parse Failures", gps_parse_failures_);
    status.add("Insufficient Data Warnings", gps_insufficient_data_warnings_);

    gps_parse_failures_ = 0;
    gps_insufficient_data_warnings_ = 0;
  }

  void DataDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

    double period = diagnostic_updater_.getPeriod();
    double measured_rate = measurement_count_ / period;

    if (measured_rate < 0.5 * expected_rate_) {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "Insufficient Data Rate");
      NODELET_ERROR("insufficient data rate <%s>: %lf < %lf", hw_id_.c_str(),
                    measured_rate, expected_rate_);
    } else if (measured_rate < 0.95 * expected_rate_) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Insufficient Data Rate");
      NODELET_WARN("insufficient data rate <%s>: %lf < %lf", hw_id_.c_str(),
                   measured_rate, expected_rate_);
    }

    status.add("Measurement Rate (Hz)", measured_rate);

    measurement_count_ = 0;
  }

  void RateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "Nominal Publish Rate");

    double elapsed = (ros::Time::now() - last_published_).toSec();
    bool gap_detected = false;
    if (elapsed > 2.0 / expected_rate_) {
      publish_rate_warnings_++;
      gap_detected = true;
    }

    if (publish_rate_warnings_ > 1 || gap_detected) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Insufficient Publish Rate");
      NODELET_WARN("publish rate failures detected <%s>: %d", hw_id_.c_str(),
                   publish_rate_warnings_);
    }

    status.add("Warnings", publish_rate_warnings_);

    publish_rate_warnings_ = 0;
  }

  void add_new_publish_adver() {
    ros::NodeHandle& node = getNodeHandle();
    pub_navsatfix = swri::advertise<sensor_msgs::NavSatFix>(node, "navsat/fix",
                                                            1);  // had_finish
    pub_imu =
        swri::advertise<sensor_msgs::Imu>(node, "imu/data", 1);  // had_finish
    pub_raw_imu =
        swri::advertise<sensor_msgs::Imu>(node, "raw_imu", 10);  // had_finish
    pub_odom = swri::advertise<nav_msgs::Odometry>(node, "navsat/odom",
                                                   1);  // need_trans
    pub_origin = swri::advertise<geometry_msgs::Pose>(node, "navsat/origin", 1);
  }


  void creat_navsatfix(novatel_gps_msgs::BESTPOS1Ptr& ros_msg) {
    sensor_msgs::NavSatFix fix_msg;

    novatel_gps_msgs::BESTPOS1 BESTPOS;
    sensor_msgs::NavSatStatus NavSatStatus;

    double seconds;
    //ASCII Header gps_mseonds: Seconds from the beginning of the GPS reference week;accurate to the millisecond level
    // Binary Header gps_mseconds: Milliseconds from the beginning of the GPS reference week
    if (use_binary_messages_)
        seconds = ros_msg -> header.gps_week * NovatelGps::SECONDS_PER_WEEK + ros_msg -> header.gps_mseconds / 1000.0;
    else
        seconds = ros_msg -> header.gps_week * NovatelGps::SECONDS_PER_WEEK + ros_msg -> header.gps_mseconds;

    // std::cout.setf(std::ios::fixed);
    // std::cout << "best_pose_mseconds = " << ros_msg -> header.gps_mseconds / 1000.0 << std::endl;
    double unix_sec = gps2unix(seconds);
    fix_msg.header.stamp = NovatelGps::rosTime(ros::WallTime(unix_sec));
    fix_msg.header.frame_id = base_frame;
    fix_msg.status.service = NavSatStatus.SERVICE_GPS;

    std::map<int, int> position_type_to_status;
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_NONE, NavSatStatus.STATUS_NO_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_FIXED, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_FIXEDHEIGHT, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_FLOATCONV, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_WIDELANE, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_NARROWLANE, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_DOPPLER_VELOCITY, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_SINGLE, NavSatStatus.STATUS_NO_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_PSRDIFF, NavSatStatus.STATUS_NO_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_WAAS, NavSatStatus.STATUS_GBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_PROPAGATED, NavSatStatus.STATUS_NO_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_OMNISTAR, NavSatStatus.STATUS_SBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_L1_FLOAT, NavSatStatus.STATUS_GBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_IONOFREE_FLOAT, NavSatStatus.STATUS_GBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_NARROW_FLOAT, NavSatStatus.STATUS_GBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_L1_INT, NavSatStatus.STATUS_GBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_WIDE_INT, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_NARROW_INT, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_RTK_DIRECT_INS, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_INS_SBAS, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_INS_PSRSP, NavSatStatus.STATUS_NO_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_INS_PSRDIFF, NavSatStatus.STATUS_NO_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_INS_RTKFLOAT, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_INS_RTKFIXED, NavSatStatus.STATUS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_INS_OMNISTAR, NavSatStatus.STATUS_GBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_INS_OMNISTAR_HP, NavSatStatus.STATUS_GBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_INS_OMNISTAR_XP, NavSatStatus.STATUS_GBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_OMNISTAR_HP, NavSatStatus.STATUS_SBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_OMNISTAR_XP, NavSatStatus.STATUS_SBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_PPP_CONVERGING, NavSatStatus.STATUS_SBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_PPP, NavSatStatus.STATUS_SBAS_FIX));
    position_type_to_status.insert(
        std::pair<int, int>(BESTPOS.POSITION_TYPE_INS_PPP_CONVERGING,
                            NavSatStatus.STATUS_SBAS_FIX));
    position_type_to_status.insert(std::pair<int, int>(
        BESTPOS.POSITION_TYPE_INS_PPP, NavSatStatus.STATUS_SBAS_FIX));

    std::map<int, int>::iterator it =
        position_type_to_status.find(ros_msg->position_type);
    if (it != position_type_to_status.end()) {
      fix_msg.status.status = (*it).second;

    } else {
      fix_msg.status.status = NavSatStatus.STATUS_NO_FIX;
    }

    if (rtk_temp_status != ros_msg->position_type &&
        (ros_msg->position_type >= 0 && ros_msg->position_type <= 74)) {
      ROS_INFO_STREAM(
          "RTK current status : "
          << novatel_gps_driver::RTK_POSITION_TYPES[ros_msg->position_type]);
      rtk_temp_status = ros_msg->position_type;
    }

    // Position in degrees.
    fix_msg.latitude = ros_msg->latitude;
    fix_msg.longitude = ros_msg->longitude;

    // Altitude in metres.
    fix_msg.altitude = ros_msg->altitude + ros_msg->undulation;
    fix_msg.position_covariance[0] = std::pow(2, ros_msg->latitude_std);
    fix_msg.position_covariance[4] = std::pow(2, ros_msg->longitude_std);
    fix_msg.position_covariance[8] = std::pow(2, ros_msg->altitude_std);

    fix_msg.position_covariance_type = fix_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;

    pub_navsatfix.publish(fix_msg);

    ROS_DEBUG("publish fix data");
  }


  void creat_imu_pac(novatel_gps_msgs::CORRIMUDATA1Ptr& ros_msg) {
    sensor_msgs::Imu Imu_msg;

    double IMU_ANGULAR_VEL_COVAR[9] = {1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3};
    double IMU_ACCEL_COVAR[9] = {1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3};

    double seconds = ros_msg -> gps_week * NovatelGps::SECONDS_PER_WEEK + ros_msg -> gps_seconds;
    double unix_sec = gps2unix(seconds);
    Imu_msg.header.stamp = NovatelGps::rosTime(ros::WallTime(unix_sec));
    Imu_msg.header.frame_id = base_frame;

    Imu_msg.orientation.x = orientation[0];
    Imu_msg.orientation.y = orientation[1];
    Imu_msg.orientation.z = orientation[2];
    Imu_msg.orientation.w = orientation[3];

    memcpy(&Imu_msg.orientation_covariance, &orientation_covariance[0],
           orientation_covariance.size() * sizeof(double));

    Imu_msg.angular_velocity.x = ros_msg->pitch_rate * imu_sample_rate_;
    Imu_msg.angular_velocity.y = ros_msg->roll_rate * imu_sample_rate_;
    Imu_msg.angular_velocity.z = ros_msg->yaw_rate * imu_sample_rate_;

    memcpy(&Imu_msg.angular_velocity_covariance, IMU_ANGULAR_VEL_COVAR,
           9 * sizeof(double));

    // Linear acceleration (m/s^2)
    Imu_msg.linear_acceleration.x = ros_msg->x_accel * imu_sample_rate_;
    Imu_msg.linear_acceleration.y = ros_msg->y_accel * imu_sample_rate_;
    Imu_msg.linear_acceleration.z = ros_msg->z_accel * imu_sample_rate_;

    memcpy(&Imu_msg.linear_acceleration_covariance, IMU_ACCEL_COVAR,
           9 * sizeof(double));

    pub_imu.publish(Imu_msg);
    ROS_DEBUG("publish Imu data");
  }

  void create_raw_imu_msgs(novatel_gps_msgs::RAWIMU1Ptr& raw_imu_msg) {
    sensor_msgs::Imu Imu_msg;

    //G320N
/*  Gyroscope Scale Factor
    (0.008/65536)/125 deg/s/LSB
    Acceleration Scale Factor
    (0.200/65536)/125 mG/s/LSB
    1 G = 9.8065 m/s2
*/
    double gyro_scale = 1.7044230976507124e-11 * imu_sample_rate_;
    double accel_scale = 2.3929443359375006e-10 * imu_sample_rate_;

    double IMU_ANGULAR_VEL_COVAR[9] = {1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3};
    double IMU_ACCEL_COVAR[9] = {1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3};

    double seconds = raw_imu_msg -> gps_week * NovatelGps::SECONDS_PER_WEEK + raw_imu_msg -> gps_seconds;
    double unix_sec = gps2unix(seconds);
    Imu_msg.header.stamp = NovatelGps::rosTime(ros::WallTime(unix_sec));
    Imu_msg.header.frame_id = base_frame;

    Imu_msg.angular_velocity.x = raw_imu_msg->x_angle_change * gyro_scale;
    Imu_msg.angular_velocity.y = -raw_imu_msg->y_angle_change_neg * gyro_scale;
    Imu_msg.angular_velocity.z = raw_imu_msg->z_angle_change * gyro_scale;

    memcpy(&Imu_msg.angular_velocity_covariance, IMU_ANGULAR_VEL_COVAR,
           9 * sizeof(double));

    // Linear acceleration (m/s^2)
    Imu_msg.linear_acceleration.x = raw_imu_msg->x_velocity_change * accel_scale;
//    std::cout.setf(std::ios::fixed);
//    std::cout << Imu_msg.linear_acceleration.x << std::endl;
    Imu_msg.linear_acceleration.y = -raw_imu_msg->y_velocity_change_neg * accel_scale;
    Imu_msg.linear_acceleration.z = raw_imu_msg->z_velocity_change * accel_scale;

    memcpy(&Imu_msg.linear_acceleration_covariance, IMU_ACCEL_COVAR,
           9 * sizeof(double));

    pub_raw_imu.publish(Imu_msg);
    ROS_DEBUG("publish Imu data");
  }

  void creat_odom_pose_msg(novatel_gps_msgs::INSPVA1Ptr& ros_msg) {
    /************************************************************************************/
    double northing, easting;
    std::string zone;

    gps_common::LLtoUTM(ros_msg->latitude, ros_msg->longitude, northing,
                        easting, zone);

    ROS_DEBUG("publish origin data %f:%f", ros_msg->latitude,
              ros_msg->longitude);

    if (!odom_init && zero_start) {
      origin.position.x = easting;
      origin.position.y = northing;
      origin.position.z = ros_msg->altitude;
      pub_origin.publish(origin);
      ROS_DEBUG("publish origin data");
      odom_init = true;
    }

    ins_status = novatel_gps_driver::INS_STATUS[ros_msg->ins_status];

    if (ins_status != temp_status) {
      temp_status = ins_status;
      ROS_INFO_STREAM("current novatel inspva status is : " << ins_status);
    }

    nav_msgs::Odometry odom;
    double seconds = ros_msg -> gps_week * NovatelGps::SECONDS_PER_WEEK + ros_msg -> gps_seconds;
    double unix_sec = gps2unix(seconds);
    odom.header.stamp = NovatelGps::rosTime(ros::WallTime(unix_sec));
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_frame;
    odom.pose.pose.position.x = easting - origin.position.x;
    odom.pose.pose.position.y = northing - origin.position.y;
    odom.pose.pose.position.z = ros_msg->altitude - origin.position.z;

    // Orientation
    // Save this on an instance variable, so that it can be published
    // with the IMU message as well.

    Eigen::Quaterniond m;
    m = Eigen::AngleAxisd((90.0 -ros_msg->azimuth) / 180 * M_PI,
                          Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-ros_msg->pitch / 180 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ros_msg->roll / 180 * M_PI,
                          Eigen::Vector3d::UnitX());

    odom.pose.pose.orientation.x = orientation[0] = m.x();
    odom.pose.pose.orientation.y = orientation[1] = m.y();
    odom.pose.pose.orientation.z = orientation[2] = m.z();
    odom.pose.pose.orientation.w = orientation[3] = m.w();

    odom.pose.covariance[0] = position_covariance[0];
    odom.pose.covariance[7] = position_covariance[4];
    odom.pose.covariance[14] = position_covariance[8];
    odom.pose.covariance[21] = orientation_covariance[0];
    odom.pose.covariance[28] = orientation_covariance[4];
    odom.pose.covariance[35] = orientation_covariance[8];

    odom.twist.twist.linear.x = ros_msg->east_velocity;
    odom.twist.twist.linear.y = ros_msg->north_velocity;
    odom.twist.twist.linear.z = ros_msg->up_velocity;

    double TWIST_COVAR[36] = {1, 0, 0, 0, 0,   0, 0, 1, 0, 0,   0, 0,
                              0, 0, 1, 0, 0,   0, 0, 0, 0, 0.1, 0, 0,
                              0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,   0, 0.1};
    TWIST_COVAR[0] = linear_velocity_covariance[0];
    TWIST_COVAR[7] = linear_velocity_covariance[4];
    TWIST_COVAR[14] = linear_velocity_covariance[8];
    memcpy(&odom.twist.covariance, TWIST_COVAR, 36 * sizeof(double));
    pub_odom.publish(odom);
    ROS_DEBUG("publish odom data");
  }

  void insstdev_handler(novatel_gps_msgs::INSSTDEV1Ptr& ros_msg) {
    orientation_covariance[0] = std::pow(2, ros_msg->pitch_std);
    orientation_covariance[4] = std::pow(2, ros_msg->roll_std);
    orientation_covariance[8] = std::pow(2, ros_msg->azimuth_std);

    position_covariance[0] = std::pow(2, ros_msg->latitude_std);
    position_covariance[4] = std::pow(2, ros_msg->longitude_std);
    position_covariance[8] = std::pow(2, ros_msg->altitude_std);

    linear_velocity_covariance[0] = std::pow(2, ros_msg->east_velocity_std);
    linear_velocity_covariance[4] = std::pow(2, ros_msg->north_velocity_std);
    linear_velocity_covariance[8] = std::pow(2, ros_msg->up_velocity_std);
  }

  void TEST_data() {
#if 1
    std::string sentence_str =
        "#INSPVAA,COM1,0,31.0,FINESTEERING,1264,144088.000,02040000,"
        "5615,1541;1264,144088.002284950,51.116827527,-114.037738908,401."
        "191547167,354.846489850,"
        "108.429407241,-10.837482850,1.116219952,-3.476059035,7.372686190,INS_"
        "ALIGNMENT_COMPLETE*a2913d36\r\n";
    gps_.ProcessData_new(sentence_str);
    Check_new_DeviceForData();

    std::string sentence_str1 =
        "#RAWIMUA,COM3,0,72.0,FINESTEERING,2038,117867.800,1a008000,fa9a,14968;2038,117867.792005000,f7f7fe00,-325882330,-39071383,104462335,-240437176,7815616,-7423919*a02e612f\r\n";
    gps_.ProcessData_new(sentence_str1);
    Check_new_DeviceForData();
    std::string path = ros::package::getPath("novatel_gps_driver");

    if (gps_.Connect(path + "/test/gpgga-gprmc-bestpos.pcap",
                     novatel_gps_driver::NovatelGps::PCAP)) {
      ROS_DEBUG("bestpos_data1111");
      gps_.ProcessData();
      Check_new_DeviceForData();
    }

    if (gps_.Connect(path + "/test/corrimudata.pcap",
                     novatel_gps_driver::NovatelGps::PCAP)) {
      ROS_DEBUG("corrimudata22222");
      gps_.ProcessData();
      Check_new_DeviceForData();
    }
#endif
  }
};
}  // namespace novatel_gps_driver

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(novatel_gps_driver, NovatelGpsNodelet)
