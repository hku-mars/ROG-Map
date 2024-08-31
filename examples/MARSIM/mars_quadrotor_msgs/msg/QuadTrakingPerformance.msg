# Header
std_msgs/Header header
# MPC finite state machine state
std_msgs/Int64 fsm_state_id
std_msgs/String fsm_state
# Solving time consuming
float64 mpc_solve_time
float64 suggest_hover_percentage

QuadState a1_reference
QuadState a2_feedback
QuadState a3_error
QuadCommand command




