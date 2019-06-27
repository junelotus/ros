# 0 0x00000001   0 = Normal, 1 = Sensor Failure
# 1 0x00000002 
# 2 0x00000004 
# 3 0x00000008 
# 4 0x00000010 
# 5 0x00000020 
# 6 0x00000040 
# 7 0x00000080 
####################################################################
# 8 0x00000100 
# 9 0x00000200     Accel Z - New Data       New Data = 1, No Data = 0
# 10 0x00000400    Accel Y - New Data       New Data = 1, No Data = 0
# 11 0x00000800    Accel X - New Data       New Data = 1, No Data = 0
# 12 0x00001000    Gyro Z - New Data        New Data = 1, No Data = 0
# 13 0x00002000    Gyro Y - New Data        New Data = 1, No Data = 0
# 14 0x00004000    Gyro X - New Data        New Data = 1, No Data = 0
# 15 0x00008000    Temperature - New Data   New Data = 1, No Data = 0
####################################################################
# 16 0x00010000
# 17 0x00020000
# 18 0x00040000
# 19 0x00080000
# 20 0x00100000
# 21 0x00200000
# 22 0x00400000    IMU Temperature reading as follows: Temperature = [(-0.0037918 * (A - 2634)) + 25] Celsius A: Temperature Sensor output data (decimal)
# 23 0x00800000
# 24 0x01000000
# 25 0x02000000
# 26 0x04000000
# 27 0x08000000
# 28 0x10000000
# 29 0x20000000
# 30 0x40000000
# 31 0x80000000


novatel_gps_msgs/CommonHeader header

uint32 gps_week
float64 gps_seconds

uint32 imu_status

int32 z_velocity_change      # change in velocity along z axis.
int32 y_velocity_change_neg  # -change in velocity along y axis.
int32 x_velocity_change      # change in velocity along x axis.
int32 z_angle_change         # change in angle around z axis.
int32 y_angle_change_neg     # -change in angle around y axis.
int32 x_angle_change         # change in angle around x axis.
