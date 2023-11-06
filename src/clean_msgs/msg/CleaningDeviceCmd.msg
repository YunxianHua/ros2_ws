#topic name: /device_manager_cmd

uint8 side_brush_ctrl                       #边刷电机状态控制   0：不使能；1：使能；
uint8 side_brush_target_rpm                 #边刷电机目标转速
uint8 scroll_brush_pushrod_state_ctrl       #滚刷推杆电机状态控制
uint8 scroll_brush_pushrod_target_pressure  #滚刷推杆电机对地压力
uint8 scroll_brush_state_ctrl               #滚刷电机状态控制
uint8 scroll_brush_target_rpm               #滚刷电机目标转速

uint8  suction_fan_state_ctrl                #吸风电机状态控制
uint16 suction_fan_target_rpm               #吸风电机目标转速高位
uint8  suction_pad_pushrod_state_ctrl       #吸水扒推杆电机状态控制
uint8  sprinkler_valve_state_ctrl           #洒水电磁阀状态控制
uint8  sprinkler_valve_target_flow          #洒水电磁阀目标流量

uint8 sewage_valve_state_ctrl               #污水阀状态控制
uint8 self_cleaning_pump_state_ctrl         #自清洁水泵状态控制
uint8 self_cleaning_pump_target_flow        #自清洁泵目标流量
uint8 self_cleaning_cycles_number           #自清洁冲洗次数