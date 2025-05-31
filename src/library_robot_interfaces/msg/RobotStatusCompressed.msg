std_msgs/Header header
float32 pose_x
float32 pose_y
float32 pose_theta
float32 velocity_linear
float32 velocity_angular
float32 battery_percentage  # 0.0 到 100.0
string robot_state_str      # TaskManager维护的机器人状态字符串, e.g., "idle", "mapping", "error_localization_lost"
string error_message        # 当前错误信息 (如果有)
bool is_emergency_stopped
int32 active_task_id_rails # 当前正在执行的Rails Task ID (0 如果没有)
int32 active_map      # ROS中当前加载的地图的ID (0 如果没有)