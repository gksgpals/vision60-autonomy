uint8 UNKNOWN=0
uint8 NORMAL=1
uint8 DEGRADED=2
uint8 LOST=3

std_msgs/Header header
string mission_id
uint8 state
string channel
bool connected
float32 signal_strength_dbm
float32 snr_db
float32 packet_loss_ratio
float32 latency_ms
builtin_interfaces/Time last_heartbeat
