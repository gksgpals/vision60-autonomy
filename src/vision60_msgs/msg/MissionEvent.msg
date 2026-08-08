uint8 INFO=0
uint8 WARNING=1
uint8 ERROR=2
uint8 CRITICAL=3

std_msgs/Header header
string mission_id
string event_id
string event_type
uint8 severity
geometry_msgs/Pose pose
string source_id
string description
float32 confidence
