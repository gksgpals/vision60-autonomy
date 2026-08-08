uint8 IDLE=0
uint8 PENDING=1
uint8 IN_PROGRESS=2
uint8 COMPLETE=3
uint8 FAILED=4

std_msgs/Header header
string mission_id
uint8 state
uint32 total_items
uint32 synchronized_items
uint32 failed_items
float32 progress_ratio
string detail
