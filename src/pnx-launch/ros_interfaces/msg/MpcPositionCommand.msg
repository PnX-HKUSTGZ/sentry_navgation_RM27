std_msgs/Header header
builtin_interfaces/Time planning_stamp
ros_interfaces/PositionCommand[] cmds
uint32 mpc_horizon
uint8 command_flag
uint8 NORMAL_COMMAND = 1
uint8 RECOVERY_COMMAND = 2
uint8 BRAKING_COMMAND = 3
uint8 BLOCK_COMMAND = 137
