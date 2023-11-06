uint8 TYPE_UNKNOWN = 0     # 未知类型，表示不需要实际导航，不用参照pose的值
uint8 TYPE_NAVI = 1        # 导航点
uint8 TYPE_SUPPLY = 2      # 补给站
uint8 TYPE_DOCK = 3        # 停靠点

uint32 type
geometry_msgs/PoseStamped pose