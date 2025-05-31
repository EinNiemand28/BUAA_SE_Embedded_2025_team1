#!/usr/bin/env python3
import rospy
import websocket # 使用 websocket-client 库
import threading
import json
import time
import uuid

from std_msgs.msg import String
from library_robot_interfaces.msg import TaskDirective, TaskFeedback, RobotStatusCompressed

# --- 配置 ---
RAILS_APP_HOST = "localhost:3000"
RAILS_APP_SCHEME = "ws"
WEBSOCKET_URL = f"{RAILS_APP_SCHEME}://{RAILS_APP_HOST}/cable"
API_KEY = "7ad0bbbdf00c5cbe87799355200f212ed329030028fd3ccd51524e461adf2c31" # 从你的.env文件同步

# Rails ActionCable Channel 名称
RAILS_COMMAND_RECEPTION_CHANNEL_CLASS = "RosCommsSubscriberChannel" # ROS节点订阅这个Channel类
RAILS_FEEDBACK_DISPATCH_CHANNEL_CLASS = "RobotFeedbackChannel"     # ROS节点向这个Channel类的方法发送消息

# ROS Topics
TM_DIRECTIVE_TOPIC = "/task_manager/directive" # WSRB publishes, TM subscribes
TM_FEEDBACK_TOPIC = "/task_manager/feedback"   # TM publishes, WSRB subscribes
ROBOT_STATUS_TOPIC = "/robot_core/status_compressed" # TM (or other core node) publishes, WSRB subscribes

class WebRobotBridgeNode:
    def __init__(self):
        # 注意：当通过launch文件启动时，不要调用rospy.init_node()
        # rospy.init_node('web_robot_bridge_node', anonymous=True)
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing Web-ROS Bridge...")

        self.ws_app = None
        self.ws_connected = False
        self.ws_thread = None
        self.heartbeat_thread = None
        self.connect_lock = threading.Lock()
        self.client_id = str(uuid.uuid4()) # 用于调试，如果Rails端需要区分不同bridge实例

        # ROS Publishers and Subscribers
        self.task_directive_pub = rospy.Publisher(TM_DIRECTIVE_TOPIC, TaskDirective, queue_size=20)
        rospy.Subscriber(TM_FEEDBACK_TOPIC, TaskFeedback, self._tm_feedback_callback, queue_size=20)
        rospy.Subscriber(ROBOT_STATUS_TOPIC, RobotStatusCompressed, self._robot_status_callback, queue_size=20)

        self._start_websocket_manager()
        rospy.loginfo(f"[{self.node_name}] Bridge Initialized (Client ID: {self.client_id}). Waiting for WebSocket connection.")

    def _start_websocket_manager(self):
        if self.ws_thread and self.ws_thread.is_alive():
            rospy.logwarn(f"[{self.node_name}] WebSocket manager thread already running.")
            return
        self.ws_thread = threading.Thread(target=self._websocket_connection_loop)
        self.ws_thread.daemon = True
        self.ws_thread.start()
        rospy.loginfo(f"[{self.node_name}] WebSocket manager thread started.")

    def _websocket_connection_loop(self):
        retry_interval = 5 # seconds
        while not rospy.is_shutdown():
            with self.connect_lock:
                if not self.ws_connected:
                    try:
                        rospy.loginfo(f"[{self.node_name}] Attempting WebSocket connection to: {WEBSOCKET_URL}")
                        headers = {
                            "X-Robot-API-Key": API_KEY, 
                            "X-Robot-Client-ID": self.client_id
                        }
                        websocket.enableTrace(True) # 取消注释以获取详细的WS帧日志
                        self.ws_app = websocket.WebSocketApp(
                            WEBSOCKET_URL, 
                            header=headers,
                            on_open=lambda ws: self._on_ws_open(ws),
                            on_message=lambda ws, msg: self._on_ws_message(ws, msg),
                            on_error=lambda ws, err: self._on_ws_error(ws, err),
                            on_close=lambda ws, code, msg: self._on_ws_close(ws, code, msg)
                        )
                        self.ws_app.run_forever(ping_interval=20, ping_timeout=5) # run_forever会阻塞
                        rospy.loginfo(f"[{self.node_name}] ws_app.run_forever() exited cleanly.")
                    except Exception as e:
                        rospy.logerr(f"[{self.node_name}] WebSocket loop error: {e}")
                    finally:
                        self.ws_connected = False # 标记为未连接以触发重连
                        if self.ws_app:
                            self.ws_app.close()
                            self.ws_app = None
                        self._stop_heartbeat() # 连接断开时停止自定义心跳（如果使用）
            
            if not self.ws_connected and not rospy.is_shutdown():
                rospy.loginfo(f"[{self.node_name}] WebSocket disconnected. Retrying in {retry_interval}s...")
                time.sleep(retry_interval)
        rospy.loginfo(f"[{self.node_name}] WebSocket connection loop terminated due to ROS shutdown.")

    def _on_ws_open(self, ws):
        rospy.loginfo(f"[{self.node_name}] WebSocket connection successfully opened.")
        self.ws_connected = True
        try:
            # 1. 订阅Rails端用于接收ROS指令的Channel实例 (RosCommsSubscriberChannel)
            #    这个Channel在Rails端会 stream_from "ros_comms_channel"
            comms_identifier = json.dumps({"channel": RAILS_COMMAND_RECEPTION_CHANNEL_CLASS})
            ws.send(json.dumps({"command": "subscribe", "identifier": comms_identifier}))
            rospy.loginfo(f"[{self.node_name}] Sent subscribe request for command reception channel: '{RAILS_COMMAND_RECEPTION_CHANNEL_CLASS}'")

            # 2. 订阅Rails端用于ROS发送反馈的Channel实例 (RobotFeedbackChannel)
            #    这样当WSRB发送带有此identifier的message时，Rails知道路由给哪个Channel对象
            # feedback_identifier = json.dumps({"channel": RAILS_FEEDBACK_DISPATCH_CHANNEL_CLASS})
            # ws.send(json.dumps({"command": "subscribe", "identifier": feedback_identifier}))
            # rospy.loginfo(f"[{self.node_name}] Sent subscribe request for feedback dispatch channel: '{RAILS_FEEDBACK_DISPATCH_CHANNEL_CLASS}'")
            # 不需要订阅，因为Rails端会自动处理

        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error during _on_ws_open while sending subscribe messages: {e}")
            self.ws_connected = False # 标记错误，以便外层循环重连
            ws.close() # 主动关闭连接
            return
        
        # self._start_heartbeat() # WebSocketApp的ping_interval参数可以处理底层PING

    def _on_ws_message(self, ws, message_str):
        rospy.logdebug(f"[{self.node_name}] Raw message from Rails: {message_str}")
        try:
            data = json.loads(message_str)
            msg_type = data.get("type")

            if msg_type == "welcome": rospy.loginfo(f"[{self.node_name}] WebSocket session established (welcome)."); return
            if msg_type == "ping": rospy.logdebug(f"[{self.node_name}] Ping from server (ts: {data.get('message')})."); return
            if msg_type == "confirm_subscription": rospy.loginfo(f"[{self.node_name}] Subscription confirmed for: {data.get('identifier')}"); return
            if msg_type == "reject_subscription": rospy.logerr(f"[{self.node_name}] Subscription REJECTED for: {data.get('identifier')}."); return

            # 处理从Rails RosCommsSubscriberChannel (stream_from "ros_comms_channel") 收到的应用指令
            # Rails端发送的广播消息，其identifier会是RosCommsSubscriberChannel的
            expected_identifier = json.dumps({"channel": RAILS_COMMAND_RECEPTION_CHANNEL_CLASS})
            if data.get("identifier") != expected_identifier:
                rospy.logwarn(f"[{self.node_name}] Message from unexpected identifier: {data.get('identifier')}. Expected: {expected_identifier}. Ignoring.")
                return

            app_message = data.get("message") # 这是Rails broadcast("ros_comms_channel", ACTUAL_MESSAGE)中的ACTUAL_MESSAGE
            if not isinstance(app_message, dict):
                rospy.logwarn(f"[{self.node_name}] 'message' field from Rails broadcast is not a dict or missing. Data: {app_message}")
                return

            command_type = app_message.get("command_type") # e.g., MOVE, EMERGENCY_STOP, TASK_CANCEL, SPECIFIC_TASK_EXECUTE, etc.
            task_info_dict = app_message.get("task")   # For TASK_EXECUTE
            payload_dict = app_message.get("payload") # For MOVE, TASK_CANCEL, etc.

            rospy.loginfo(f"[{self.node_name}] Parsed app command from Rails '{RAILS_COMMAND_RECEPTION_CHANNEL_CLASS}': Type='{command_type}'")

            directive = TaskDirective()
            directive.header.stamp = rospy.Time.now()
            directive.header.frame_id = self.client_id # WSRB的ID
            directive.command_type = command_type or "UNKNOWN_RAILS_COMMAND"

            # 如果没有task_info_dict, 则认为是即时控制指令; 可根据需要扩展
            if task_info_dict:
                directive.task_id_rails = str(task_info_dict.get("id", "0"))
                directive.task_type_rails = task_info_dict.get("type", command_type) # 如果type缺失，用command_type
                directive.task_priority = task_info_dict.get("priority", 0) # Rails端任务优先级
                directive.parameters_json = json.dumps(task_info_dict.get("parameters", {}))
            else:
                directive.task_id_rails = "0"
                directive.task_type_rails = command_type
                directive.parameters_json = json.dumps(payload_dict) if payload_dict else "{}"
            
            self.task_directive_pub.publish(directive)
            rospy.loginfo(f"[{self.node_name}] Published TaskDirective to '{TM_DIRECTIVE_TOPIC}': Cmd='{directive.command_type}', TaskType='{directive.task_type_rails}', TaskID='{directive.task_id_rails}'")

        except json.JSONDecodeError:
            rospy.logerr(f"[{self.node_name}] Failed to decode JSON from Rails: {message_str}")
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing message from Rails: {e}\nMsg: {message_str}", exc_info=True)

    def _on_ws_error(self, ws, error):
        rospy.logerr(f"[{self.node_name}] WebSocket error: {error}")
        # ws.close() 会在run_forever中处理错误并退出循环，然后外层循环会重连

    def _on_ws_close(self, ws, close_status_code, close_msg):
        rospy.logwarn(f"[{self.node_name}] WebSocket connection closed. Code: {close_status_code}, Msg: {close_msg}")
        self.ws_connected = False
        self._stop_heartbeat() # 如果有自定义心跳

    def _send_to_rails_channel_action(self, rails_action_name, payload_dict_for_action):
        if not self.ws_connected or not self.ws_app:
            rospy.logwarn(f"[{self.node_name}] WS not connected. Cannot send to Rails: Action='{rails_action_name}'")
            return False
        
        # 确保payload是字典
        if not isinstance(payload_dict_for_action, dict):
            rospy.logerr(f"[{self.node_name}] Invalid payload_dict_for_action (not a dict) for action {rails_action_name}: {type(payload_dict_for_action)}")
            return False

        message_to_rails = {
            "command": "message",
            "identifier": json.dumps({"channel": RAILS_FEEDBACK_DISPATCH_CHANNEL_CLASS}),
            "data": json.dumps({
                "action": rails_action_name, # Rails RobotFeedbackChannel上的方法
                "payload": payload_dict_for_action # 传递给该方法的数据
            })
        }
        try:
            self.ws_app.send(json.dumps(message_to_rails))
            # 简化日志，避免打印完整payload，特别是包含图像数据时
            log_summary = f"TaskID: {payload_dict_for_action.get('task_id', 'N/A')}" if 'task_id' in payload_dict_for_action else "General Update"
            rospy.loginfo(f"[{self.node_name}] Sent to Rails Action '{rails_action_name}'. Summary: {log_summary}")
            return True
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error sending action '{rails_action_name}' to Rails: {e}")
            return False

    def _start_heartbeat(self): # WebSocketApp 内置了 ping_interval, 此自定义心跳可能不再严格需要
        pass # 如果 WebSocketApp 的 ping_interval 足够，则不需要应用层心跳

    def _stop_heartbeat(self): # 同上
        pass

    # --- ROS Topic Callbacks ---
    def _tm_feedback_callback(self, tm_feedback_msg): # Type: TaskFeedback
        rospy.loginfo(f"[{self.node_name}] Received TaskFeedback from TM: TaskID='{tm_feedback_msg.task_id_rails}', Action='{tm_feedback_msg.feedback_action_rails}'")
        try:
            payload_for_rails = json.loads(tm_feedback_msg.feedback_payload_json) if tm_feedback_msg.feedback_payload_json else {}
            self._send_to_rails_channel_action(tm_feedback_msg.feedback_action_rails, payload_for_rails)
        except json.JSONDecodeError:
            rospy.logerr(f"[{self.node_name}] Failed to decode JSON from TM feedback_payload_json: {tm_feedback_msg.feedback_payload_json}")
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error in _tm_feedback_callback: {e}", exc_info=True)

    def _robot_status_callback(self, rs_compressed_msg): # Type: RobotStatusCompressed
        rospy.logdebug(f"[{self.node_name}] Received RobotStatusCompressed from TM: State='{rs_compressed_msg.robot_state_str}'")
        try:
            payload_for_rails = {
                "pose": {"x": rs_compressed_msg.pose_x, "y": rs_compressed_msg.pose_y, "theta": rs_compressed_msg.pose_theta},
                "velocity": {"linear": rs_compressed_msg.velocity_linear, "angular": rs_compressed_msg.velocity_angular},
                "battery_level": rs_compressed_msg.battery_percentage,
                "overall_status": rs_compressed_msg.robot_state_str, # 这是TM管理的核心状态
                "error_message": rs_compressed_msg.error_message if rs_compressed_msg.error_message else None,
                "is_emergency_stopped": rs_compressed_msg.is_emergency_stopped,
                "active_task_id": rs_compressed_msg.active_task_id_rails, # 当前活跃的Rails任务ID
                "active_map_id": rs_compressed_msg.active_map # 当前活跃的地图名称
                # active_task_id_rails 和 active_map 已经包含在 RobotStatus 模型中了，
                # Rails端的 RobotStatus.update_status_from_ros 会处理 overall_status, error, is_emergency_stopped
                # Rails端的 Task 模型回调会更新与任务相关的状态。
                # 所以这里发送的 overall_status 主要是给 Rails RobotStatus 一个参考。
            }
            self._send_to_rails_channel_action("update_robot_state", payload_for_rails)
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error in _robot_status_callback: {e}", exc_info=True)

if __name__ == '__main__':
    wsrb_node_instance = None
    try:
        # 当作为独立脚本运行时初始化节点
        rospy.init_node('web_robot_bridge_node', anonymous=False)
        wsrb_node_instance = WebRobotBridgeNode()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo(f"[{wsrb_node_instance.node_name if wsrb_node_instance else 'web_robot_bridge'}] Shutting down.")
    finally:
        rospy.loginfo(f"[{wsrb_node_instance.node_name if wsrb_node_instance else 'web_robot_bridge'}] Performing final cleanup.")
        if wsrb_node_instance and wsrb_node_instance.ws_app:
            try:
                wsrb_node_instance.ws_connected = False
                wsrb_node_instance.ws_app.close()
            except:
                pass
        rospy.loginfo(f"[{wsrb_node_instance.node_name if wsrb_node_instance else 'web_robot_bridge'}] Cleanup complete. Exiting.")

# --- 自定义 ROS 消息 (示例，应放在项目的 msg 目录下) ---
# library_robot_interfaces/msg/TaskDirective.msg
"""
std_msgs/Header header
string command_type          # e.g., TASK_EXECUTE, TASK_CANCEL, MOVE, EMERGENCY_STOP
string task_id_rails         # Rails Task ID (string to be safe)
string task_type_rails       # e.g., MAP_BUILD_AUTO, LOAD_MAP, or command_type for non-task commands
int32 task_priority          # Rails Task priority
string parameters_json       # JSON string of parameters for the task or command
"""

# library_robot_interfaces/msg/TaskFeedback.msg
"""
std_msgs/Header header
string feedback_action_rails    # Name of the action on Rails RobotFeedbackChannel to call
                                # e.g., update_task_progress, report_map_preview, report_task_completion
string feedback_payload_json    # JSON string of the payload for that action
"""

# library_robot_interfaces/msg/RobotStatusCompressed.msg
"""
std_msgs/Header header
float32 pose_x
float32 pose_y
float32 pose_theta
float32 velocity_linear
float32 velocity_angular
float32 battery_percentage  # 0.0 to 100.0
string robot_state_str      # e.g., "idle", "mapping", "error_localization_lost"
string error_message        # Current error message, if any
bool is_emergency_stopped
int32 active_task_id_rails  # Current active Rails Task ID, 0 if none
int32 active_map            # ID of the currently loaded map in ROS
"""