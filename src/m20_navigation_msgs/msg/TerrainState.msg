std_msgs/Header header

uint8 UNKNOWN=0
uint8 FLAT=1
uint8 STAIR_UP=2
uint8 STAIR_DOWN=3

uint8 current_mode
uint8 upcoming_mode
float32 distance_to_entry
float32 distance_to_exit
float32 confidence
bool valid
uint32 route_id
string reason
