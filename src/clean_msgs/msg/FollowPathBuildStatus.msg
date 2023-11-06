#topic name:/follow_path_build_status
uint32 INBUILD = 1                          #示教路径创建中
uint32 CLOSE_TO_START = 2                   #接近开始点
uint32 AUTO_STOP_WITH_CLOSE_TO_MID = 3      #自动停止录制：与中间点发生闭合
uint32 AUTO_CANCEL_WITH_LOC_LOST = 4        #自动取消录制: 定位丢失

uint32 status               # 示教路径创建中状态
string exec_id              # 执行ID， 开始成功后返回
geometry_msgs/Point point   # <可选>新增路径点:只有x，y有效


