uint32 BUILD_TYPE_FOLLOW = 1
uint32 BUILD_TYPE_HANDDRAWN=  2

uint32 PATH_TYPE_LINE = 1
uint32 PATH_TYPE_ZONE = 2

string id
string map_id
string name
string build_time
uint32 build_type
uint32 path_type
geometry_msgs/Point[] points    #可选: List时不返回；返回时只有x,y,没有z
string cleaning_mode_id         #可选
