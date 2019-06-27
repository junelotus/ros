# message 507
novatel_gps_msgs/CommonHeader header

int32 gps_week
float64 gps_seconds

float64 latitude
float64 longitude
float64 altitude

float64 north_velocity
float64 east_velocity
float64 up_velocity

float64 roll
float64 pitch
float64 azimuth

uint32 ins_status
uint32 INS_STATUS_INACTIVE=0
uint32 INS_STATUS_ALIGNING=1
uint32 INS_STATUS_HIGH_VARIANCE=2
uint32 INS_STATUS_SOLUTION_GOOD=3
uint32 INS_STATUS_SOLUTION_FREE=6
uint32 INS_STATUS_ALIGNMENT_COMPLETE=7
uint32 INS_STATUS_DETERMINING_ORIENTATION=8
uint32 INS_STATUS_WAITING_INITIALPOS=9

