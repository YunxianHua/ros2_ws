#topic name: /task_run_status; bm->mqttclient
uint8 TASK_STATUS_IDLE = 0              # 稳定态：空闲中
uint8 TASK_STATUS_STARTING = 1          # 过渡态：启动处理中
uint8 TASK_STATUS_RUNNING = 2          # 稳定态：运行中
uint8 TASK_STATUS_PAUSING = 3           # 过渡态：暂停处理中
uint8 TASK_STATUS_PAUSED = 4           # 稳定态：暂停中
uint8 TASK_STATUS_RESUMING = 5          # 过渡态：恢复处理中
uint8 TASK_STATUS_STOPPING = 6          # 过渡态：停止处理中


uint8 TASK_PHASE_IDLE = 0                       # 空闲阶段
uint8 TASK_PHASE_NAVIGATING = 1                 # 导航到点阶段
uint8 TASK_PHASE_CLEANING = 2                   # 清扫阶段
uint8 TASK_PHASE_DEVICE_PREPARE = 3             # 设备准备阶段
uint8 TASK_PHASE_COLLECT_SFW_IN_TASK = 4                     # 任务中集水阶段1:甩水 SFW= ShakeOffWater
uint8 TASK_PHASE_COLLECT_DS_IN_TASK = 5                      # 任务中集水阶段2:延迟吸风 DS=DelaySuction
uint8 TASK_PHASE_COLLECT_SFW_AFTER_TASK = 6                  # 任务结束后集水阶段1:甩水 SFW= ShakeOffWater
uint8 TASK_PHASE_COLLECT_DS_AFTER_TASK = 7                   # 任务接受集水阶段2:延迟吸风 DS=DelaySuction
uint8 TASK_PHASE_DS = 8                                      # 暂停中或超时未动导致:延迟吸风 DS=DelaySuction
uint8 TASK_PHASE_COLLECT_DS_FINISHED = 9                     # 延迟吸风完成 DS=DelaySuction
uint8 TASK_PHASE_NAVIGATING_AFTER_TASK = 10                  # 任务结束后：导航到点阶段

uint8 TASK_FINISHED_TYPE_UNKNOWN = 0           # 未知
uint8 TASK_FINISHED_TYPE_NORMAL = 1            # 正常结束
uint8 TASK_FINISHED_TYPE_EXCEPTION = 2         # 异常结束
uint8 TASK_FINISHED_TYPE_MANUAL = 3            # 人为结束


uint8 SUBTASK_TYPE_UNKNOWN = 0
uint8 SUBTASK_TYPE_CLEANING = 1                # 自动清洁
uint8 SUBTASK_TYPE_NAVI = 2                    # 自动导航
uint8 SUBTASK_TYPE_NAVI_IN_BC = 3              # 自动导航到补给站(断点续扫中)
uint8 SUBTASK_TYPE_COLLECT = 4                 # 集水
uint8 SUBTASK_TYPE_COLLECT_MANUAL = 5          # 手动集水
uint8 SUBTASK_TYPE_CLEAING_MANUAL = 6          # 手动清洁


string task_exe_id            # 组合任务执行ID
string task_id                # 组合任务task id
string task_name              # 组合任务名称
string task_cleaning_mode_name     # 组合任务整体清洁模式名称
string task_cleaning_mode_id
uint8 all_loop_count         # 组合任务总的要求循环次数, 0表示无限次
uint8 task_status             # 组合任务运行状态
uint8 task_phase              # 任务所处阶段
uint8 task_finished_type      # 任务结束分类
uint8 cur_loop_count          # 组合任务当前循环次数

float32 collect_auto_shakeoffwater_remain_dis   # (自动清洁后) 集水阶段1:甩水剩余距离(可变化)     单位:米 -1时忽略
int8  collect_manual_shakeoffwater_time         # (手动清洁后) 集水阶段1:甩水 总时间(不变化)      单位:秒  -1时忽略
int8  collect_delaysuction_time                 # 集水阶段2或暂停中:   延迟吸风 总时间(不变化)    单位:秒  -1时忽略

bool manual_idle_timeout      # 手动清洁/手动集水时机器人未动超时提示 true: 超时

string cur_subtask_name
uint8 cur_subtask_type

# uint8 all_subtask_count
# string cur_subtask_name
# uint8 cur_subtask_count 
# uint8 cur_subtask_progress
# string cur_cleaning_mode_name