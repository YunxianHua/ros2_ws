#topic name: /chassis_status


std_msgs/Header header
float32 left_wheel_rpm_status      #左轮rpm速度状态 [-3000,3000] 精度0.1
float32 right_wheel_rpm_status     #右轮rpm速度状态 [-3000,3000] 精度0.1

uint32 left_wheel_encoder_pos     #左轮毂编码器位置值
uint32 right_wheel_encoder_pos    #右轮毂编码器位置值
