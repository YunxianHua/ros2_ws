uint32 SUBTASK_TYPE_PATH = 1
uint32 SUBTASK_TYPE_NAVI = 2

uint32 PATH_TYPE_LINE = 1
uint32 PATH_TYPE_ZONE = 2

uint32 type                 # SubTask类型 [必填]
string map_id               # [必填] 
string path_id              # 清洁路线/清洁区域ID [必填] only when type=SUBTASK_TYPE_PATH
uint32 path_type            # 清洁类型 [选填] TaskGet返回时才使用
string cleaning_mode_id     # 清洁模式ID [选填] only when type=SUBTASK_TYPE_PATH
string point_id             # 点ID [必填] only when type=SUBTASK_TYPE_NAVI(本期不开发)
