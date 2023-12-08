cmake_minimum_required(VERSION 3.5)
project(clean_msgs)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
        "msg/AIDetections.msg"
        "msg/AIDetInfo.msg"
        "msg/AutoDriveCmd.msg"
        "msg/AutoDriveModeSwitch.msg"
        "msg/bm/BCTaskStatus.msg"
        "msg/bm/TaskReportStatus.msg"
        "msg/bm/TaskRunStatus.msg"
        "msg/bm/UIPointInfo.msg"
        "msg/ChassisStatus.msg"
        "msg/CleaningDeviceCmd.msg"
        "msg/CleanModeInfo.msg"
        "msg/CleanModuleState.msg"
        "msg/Critic.msg"
        "msg/DeviceControlRsp.msg"
        "msg/DeviceInfo.msg"
        "msg/DevModeConf.msg"
        "msg/FollowPathBuildStatus.msg"
        "msg/FusionObjectInfo.msg"
        "msg/FusionObjects.msg"
        "msg/hm/ProcessMonitorInfo.msg"
        "msg/hm/SystemMonitorInfo.msg"
        "msg/MapDataUpdateInfo.msg"
        "msg/MapInfo.msg"
        "msg/PathInfo.msg"
        "msg/PointInfo.msg"
        "msg/RobotHeartBeat.msg"
        "msg/SemanticInfo.msg"
        "msg/SpecialZoneInfo.msg"
        "msg/SubTaskInfo.msg"
        "msg/SysBaseStatus.msg"
        "msg/TrackedObstacles.msg"
        "msg/VehicleStatus.msg"
        "msg/WirelessNetwork.msg"
)

set(srv_files
        "srv/BagRecord.srv"
        "srv/bm/map/MapConfigBuild.srv"
        "srv/bm/map/MapPrepareSync.srv"
        "srv/bm/map/MapPrepareTypeTransfer.srv"
        "srv/bm/map/MapSyncFinishNotify.srv"
        "srv/bm/map/MapTypeTransfer.srv"
        "srv/bm/path/FollowPathBuild.srv"
        "srv/bm/path/HandDrawPathSave.srv"
        "srv/bm/path/PathDel.srv"
        "srv/bm/path/PathGet.srv"
        "srv/bm/path/PathList.srv"
        "srv/bm/path/PathOpr.srv"
        "srv/bm/path/PathUpdate.srv"
        "srv/bm/point/PointCreate.srv"
        "srv/bm/point/PointDel.srv"
        "srv/bm/point/PointGet.srv"
        "srv/bm/point/PointList.srv"
        "srv/bm/point/PointUpdate.srv"
        "srv/bm/task/ManualCleanCtrl.srv"
        "srv/bm/task/RemoteCtrlCmd.srv"
        "srv/bm/task/TaskCreate.srv"
        "srv/bm/task/TaskDel.srv"
        "srv/bm/task/TaskGet.srv"
        "srv/bm/task/TaskList.srv"
        "srv/bm/task/TaskRunOperation.srv"
        "srv/bm/task/TaskRunStart.srv"
        "srv/bm/task/TaskUpdate.srv"
        "srv/bm/TaskReportGet.srv"
        "srv/CleanMode.srv"
        "srv/DevelopMode.srv"
        "srv/DeviceControl.srv"
        "srv/DeviceLevel.srv"
        "srv/DeviceTimerPauseNotify.srv"
        "srv/DriveModeSwitch.srv"
        "srv/GetAgiBotVersion.srv"
        "srv/GetCleanModeInfo.srv"
        "srv/GetCollectWaterTime.srv"
        "srv/GetCurCleanMode.srv"
        "srv/mapengine/MapConfig.srv"
        "srv/mapengine/MapLoad.srv"
        "srv/NavCtrlCmd.srv"
        "srv/NetWorkConfig.srv"
        "srv/SemanticConfig.srv"
        "srv/SetCleanMode.srv"
        "srv/SlamInterface.srv"
        "srv/SoundConfig.srv"
        "srv/SoundPlay.srv"
        "srv/SpecialZoneCmd.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${srv_files}
        DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
