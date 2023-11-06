uint32 DEVICE_LEVEL_CLOSE = 1
uint32 DEVICE_LEVEL_OPEN = 2
uint32 DEVICE_LEVEL_LOW = 3
uint32 DEVICE_LEVEL_MID = 4
uint32 DEVICE_LEVEL_HIGH = 5

string TYPE_CLEAN_PRESSURE = "Clean_Pressure"
string TYPE_BRUSH_ROLL_SPEED = "Brush_Roll_Speed"
string TYPE_BRUSH_SIDE_SPEED = "Brush_Side_Speed"
string TYPE_SPRAY_LEVEL = "Spray_Level"
string TYPE_SUCTION_LEVEL = "Suction_Level"
string TYPE_CLEAN_SPEED = "Clean_Speed"
string TYPE_SQUEEGEE = "Squeegee"

string device_type      # 类型
uint32 level          # 等级（低、中、高）
float32 value         # 对应的具体数值（百分比）