std_msgs/Header header

uint8 MODE_HOLD=0
uint8 MODE_TRACK=1
uint8 MODE_RETURN=2
uint8 MODE_DISABLE=3

uint32 last_command_sequence
uint8 mode
uint32 active_joint_mask
uint16 fault_code
