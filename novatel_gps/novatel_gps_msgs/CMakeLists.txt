cmake_minimum_required(VERSION 2.8.3)
project(novatel_gps_msgs)

set(MSG_DEPS 
  std_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
)

add_service_files(
  FILES
  NovatelFRESET.srv
)

add_message_files(DIRECTORY msg FILES
  CommonHeader.msg
  Gpgga.msg
  Gpgsa.msg
  Gpgsv.msg
  Gprmc.msg
  Inscov.msg
  Inspva.msg
  Insstdev.msg
  NovatelCorrectedImuData.msg
  NovatelExtendedSolutionStatus.msg
  NovatelMessageHeader.msg
  NovatelPosition.msg
  NovatelReceiverStatus.msg
  NovatelSignalMask.msg
  NovatelVelocity.msg
  RangeInformation.msg
  Range.msg
  Satellite.msg
  Time.msg
  TrackstatChannel.msg
  Trackstat.msg
  INSPVA1.msg
  BESTPOS1.msg
  CORRIMUDATA1.msg
  INSSTDEV1.msg
  RAWIMU1.msg
)
generate_messages(DEPENDENCIES std_msgs ${MSG_DEPS})
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs roscpp ${MSG_DEPS}
)
