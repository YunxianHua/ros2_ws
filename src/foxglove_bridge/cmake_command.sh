/usr/bin/cmake  /home/coffee/ros2_ws/src/foxglove_bridge/  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX=/home/coffee/ros2_ws/install/foxglove_bridge/
/usr/bin/cmake --build   /home/coffee/ros2_ws/build/foxglove_bridge/ -- -j8 -l8
/usr/bin/cmake --install /home/coffee/ros2_ws/build/foxglove_bridge/
#/bin/bash -c "source /opt/ros/foxy/setup.bash"
#/bin/bash -c "source /home/coffee/ros2_ws/install/setup.bash"
#colcon build --base-paths /home/coffee/ros2_ws/src
