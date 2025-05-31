std_msgs/Header header
# string task_id_rails            # 此反馈相关的Rails Task ID (如果适用)
string feedback_action_rails    # 要在Rails RobotFeedbackChannel上调用的方法名
                                # 例如: "update_task_progress", "report_map_preview", "report_task_completion"
string feedback_payload_json    # JSON字符串，作为上述Rails Channel方法的payload