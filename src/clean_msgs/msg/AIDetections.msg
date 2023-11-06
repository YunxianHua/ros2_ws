#topic name: /ai_perception_node

std_msgs/Header header          # get from camera/color header
builtin_interfaces/Time stamp
uint8 camera_id                 # camera id, start from 1 
AIDetInfo[] dets                # detections