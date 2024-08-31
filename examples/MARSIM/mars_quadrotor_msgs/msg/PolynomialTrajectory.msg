Header header

# the trajectory id, starts from "1".
uint32 trajectory_id

# The type of the trajectory
# use bit to represent the type of the trajectory
# c++ code example:
# To build a type flag: if this trajectory is a POSITION_TRAJ and YAW_TRAJ, then "type = POSITION_TRAJ | YAW_TRAJ"
# To decode this type flag: bool is_position_traj = type & POSITION_TRAJ, bool is_yaw_traj = type & YAW_TRAJ
uint32 type
uint32 HEART_BEAT                  =   1    # (1<<0)
uint32 POSITION_TRAJ               =   2    # (1<<1)
uint32 YAW_TRAJ                    =   4    # (1<<2)
uint32 YAW_COMMAND                 =   8    # (1<<3)
uint32 EMER_STOP                   =   16   # (1<<4)

# the order of trajectory.
uint32 piece_num_yaw
uint32 piece_num_pos
uint32 order_pos
uint32 order_yaw

# the polynomial coecfficients of the trajectory.
# The start system wall time of the yaw trajectory
time start_WT_yaw
float64 yaw
float64 yaw_rate
float64[] coef_yaw
float64[] time_yaw
# The start system wall time of the pos trajectory
time start_WT_pos
float64[] coef_pos_x
float64[] coef_pos_y
float64[] coef_pos_z
float64[] time_pos

string debug_info
