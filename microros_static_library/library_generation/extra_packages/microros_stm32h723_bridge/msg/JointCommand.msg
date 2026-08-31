std_msgs/Header header

uint8 MODE_HOLD=0
uint8 MODE_TRACK=1
uint8 MODE_RETURN=2
uint8 MODE_DISABLE=3

uint32 sequence
uint8 mode
float32[7] position
float32[7] velocity
float32[7] kp
float32[7] kd
float32[7] effort
