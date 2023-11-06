bool clean_water_limit                          # 清水箱液位极限 0：未触发；1：触发；
bool sewage_limit                               # 污水箱液位极限 0：未触发；1：触发；
uint8 scoll_brush_pushrod_state                 #滚刷推杆电机状态 0：抬起到位；1：抬起中；2下降中；3：下降到位；
uint8 suction_pad_pushrod_state                 #吸水扒推杆电机状态 0：抬起到位；1：抬起中；2下降中；3：下降到位；
uint8 scoll_brush_state                         #滚刷电机使能状态 0：未使能；1：使能；
uint16 scoll_brush_actual_rpm                   #滚刷电机实际转速
uint8 scroll_brush_pushrod_actual_pressure      #滚刷推杆电机实际对地压力
uint8 side_brush_state                          #边刷电机使能状态   0：未使能；1：使能；
uint8 side_brush_actual_rpm                     #边刷电机实际转速
uint8 suction_fan_state                         #吸风电机使能状态
uint16  suction_fan_actual_rpm                  #吸风电机实际转速
uint8 sprinkler_valve_state                     #洒水电磁阀使能状态
uint8 sprinkler_valve_actual_flow               #洒水电磁阀目标流量
uint8 self_cleaning_pump_state                  #自清洁泵使能状态
uint8 self_cleaning_pump_actual_flow            #自清洁泵目标流量
uint8 sewage_valve_state                        #污水阀使能状态
uint8  cleanser_loaded_state                     #清洁液在位 0:正常，1：异常
uint8  cleanser_low_state                        #清洁液不足 0:正常 1:异常
uint8  clean_water_fully                         #清水满状态 0:正常 1:异常
uint8 sewage_fully_state                        #污水满状态  0:正常 1：异常
uint8 clean_water_level                         #清水箱液位 [0, 100]
uint8 sewage_level                              #污水箱液位 [0, 100]
uint8 negative_pressure                         #负压传感器
uint8 temperature1                              #温度传感器1
uint8 temperature2                              #温度传感器2
float32 clean_water_val                           #清水箱水量 单位l，小数点后面1位
