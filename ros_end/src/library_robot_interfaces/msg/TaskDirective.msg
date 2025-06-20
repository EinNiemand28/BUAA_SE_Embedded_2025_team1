std_msgs/Header header
string command_type          # 例如: "TASK_EXECUTE", "TASK_CANCEL", "MOVE", "EMERGENCY_STOP"
string task_id_rails         # Rails Task ID (如果适用，对于即时命令可以为空或特定值)
string task_type_rails       # 例如: "MAP_BUILD_AUTO", "LOAD_MAP", 或对于即时命令，同command_type
int32 task_priority         # 任务优先级，0表示最低，10表示最高
string parameters_json       # JSON字符串，包含任务或命令所需的参数