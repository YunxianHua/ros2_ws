# Describe map info message

uint8 MAP_TYPE_PRIVATE = 1
uint8 MAP_TYPE_PROJECT = 2

string map_id              # 地图唯一id标识
string name                # 地图展示名称
string building            # 地图所属楼栋
string floor               # 地图所属楼层
bool dirty                 # 地图是否同步
uint8 map_type             # 地图类型
uint32 creat_time          # 地图创建时间
uint32 cloud_ver           # 地图版本
float32 resolution         # 地图分辨率信息
geometry_msgs/Pose origin  # 地图原点
sensor_msgs/Image data     # 具体地图数据