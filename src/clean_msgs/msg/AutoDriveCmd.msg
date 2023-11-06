#topic name: /control_autodrive_cmd
uint32 DRIVE_STATUS_DEC = 0   #减速
uint32 DRIVE_STATUS_ACC = 1   #加速
uint32 DRIVE_STATUS_EB = 2    #紧急制动

uint32  target_drive_status   #目标行驶状态
float32 target_left_wheel_rpm  #目标左轮rpm速度 [-3000,3000] 精度0.1
float32 target_right_wheel_rpm  #目标右轮rpm速度 [-3000,3000] 精度0.1
