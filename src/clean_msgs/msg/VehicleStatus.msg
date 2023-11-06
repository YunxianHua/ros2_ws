#topic name: /vehicle_status

uint32  AUTODRIVE_MODE_AUTO = 0     #自动控制模式
uint32  AUTODRIVE_MODE_MANUAL = 1   #人工控制模式

bool front_crash_state          #前向碰撞触发状态  0：未触发；1：触发；
bool left_crash_state           #左向碰撞触发状态  0：未触发；1：触发；
bool right_crash_state          #右向碰撞触发状态  0：未触发；1：触发；
bool rear_crash_state           #后向碰撞触发状态  0：未触发；1：触发；

bool front_fall_state           #前向防跌落传感器状态 0：无危险；1：有跌落风险；
bool rear_fall_state            #后向防跌落传感器状态 0：无危险；1：有跌落风险；
bool key_switch_state           #钥匙开关状态  0：关；1：开；
bool proximity_switch1          #接近开关1 0：未触发；1：触发；
bool proximity_switch2          #接近开关2 0：未触发；1：触发；
bool proximity_switch3          #接近开关3 0：未触发；1：触发；
bool proximity_switch4          #接近开关4 0：无动作；1：动作中；
uint8 driver_status             #车辆行驶状态  0：正常；1：软急停；2：硬急停；
uint8 driver_mode               #车辆行驶模式  0：自动模式；1：手动模式
uint8 soc                       #电量百分比
bool  charge_state              #充电状态 false: 未充电， true:充电
clean_msgs/CleanModuleState  clean_module_state
float32 soc_val                   #电量数值 单位 kwh, 小数点后1位


