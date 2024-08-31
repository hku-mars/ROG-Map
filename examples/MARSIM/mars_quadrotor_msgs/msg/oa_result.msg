std_msgs/Header header
uint32 direction
int32 reserved
float32 tof_dist
int32 image_index
uint32 image_timestamp
int16[3] object_distance        # uint8 NUMBER_SUB_DIRECTIONS = 3, for FC
uint8 stop
uint8 failed_distance
uint8 failed_distance_app
uint8 speed_limit
uint8 alarm
int16[4] object_distance_app    # uint8 NUMBER_SUB_DIRECTIONS_FOR_APP = 4, for APP
int32 high_atti
int32 is_release_brake