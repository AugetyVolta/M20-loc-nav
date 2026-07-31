std_msgs/Header header

uint8 TERRAIN_UNKNOWN=0
uint8 TERRAIN_FLAT=1
uint8 TERRAIN_STAIR_UP=2
uint8 TERRAIN_STAIR_DOWN=3

uint8 PROFILE_FLAT=1
uint8 PROFILE_STAIR=2

uint8 SCAN_RAW=1
uint8 SCAN_TRAVERSABILITY=2

uint8 current_terrain
uint8 upcoming_terrain
float32 distance_to_entry
bool terrain_valid
uint32 route_id

bool pct_dynamic_enabled
uint8 costmap_profile
uint8 scan_profile
bool heading_guard_enabled
uint8 gait_terrain
string reason
