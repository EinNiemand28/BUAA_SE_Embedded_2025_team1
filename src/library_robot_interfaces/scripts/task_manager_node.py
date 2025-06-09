#!/usr/bin/env python3
import rospy
import threading
import json
import time
import random
import math
import base64
import io
import tf2_ros
from collections import deque
import heapq
import subprocess

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped # For AMCL
from nav_msgs.msg import Odometry # For Odom
import tf.transformations # For quaternion to euler conversion (standard in ROS1)

from library_robot_interfaces.msg import TaskDirective, TaskFeedback, RobotStatusCompressed

from mapping.srv import Start, StartResponse, Halt, HaltResponse
from navigation.srv import Goal, GoalResponse
from navigation.srv import Start as nStart, StartResponse as nStartResponse, Halt as nHalt, HaltResponse as nHaltResponse
from std_srvs.srv import Trigger, TriggerResponse, Empty
from arm_controller.srv import Place, PlaceResponse
from fetch_server.srv import Fetch, FetchResponse

AUTO_MAPPING_SERVICE_NAME = "/auto_mapping_service"
HALT_MAPPING_SERVICE_NAME = "/halt_mapping"
NAVIGATION_ENABLE_SERVICE_NAME = "/navigation_service" # 启用导航服务（包括地图的加载）
NAVIGATION_GOAL_SERVICE_NAME = "/goal_service"
NAVIGATION_HALT_SERVICE_NAME = "/halt_goal"
# GRAB_SERVICE_NAME = "/grab_service"
# PLACE_SERVICE_NAME = "/place_service"
FETCH_BOOK_SERVICE_NAME = "/fetch_service"

# ROS Topics
TM_DIRECTIVE_TOPIC = "/task_manager/directive" # TM subscribes
TM_FEEDBACK_TOPIC = "/task_manager/feedback"   # TM publishes
ROBOT_STATUS_TOPIC = "/robot_core/status_compressed" # TM publishes this
CMD_VEL_TOPIC = "/cmd_vel"                     # TM publishes to control robot
ODOM_TOPIC = "/odom"                           # TM subscribes
AMCL_POSE_TOPIC = "/amcl_pose"                 # TM subscribes (optional, if AMCL is used)

# TF Frames (可在__init__中获取参数)
MAP_FRAME_ID = "map"
ODOM_FRAME_ID = "odom"  
BASE_FRAME_ID = "base_link"

class TaskInfo:
    def __init__(self, directive):
        self.directive = directive
        self.task_id = directive.task_id_rails if directive.task_id_rails else "0"
        self.task_type = directive.task_type_rails if directive.task_type_rails else "unknown"
        self.priority = directive.task_priority if directive.task_priority else 0
        self.params = json.loads(directive.parameters_json) if directive.parameters_json else {}
        self.queued_time = rospy.Time.now()
        self.stop_event = threading.Event() # 用于取消任务的事件
        # self.is_cancelled = False

    def __lt__(self, other):
        # 数字越大优先级越高, 同优先级按时间先后
        if self.priority != other.priority:
            return self.priority > other.priority
        return self.queued_time < other.queued_time

    def set_cancelled(self):
        self.is_cancelled = True
        if self.stop_event:
            self.stop_event.set()

import numpy as np
import cv2
from PIL import Image
import os

def convert_pgm_to_image_base64(pgm_file_path):
    try:
        map_image = cv2.imread(pgm_file_path, cv2.IMREAD_GRAYSCALE)

        if map_image is None:
            rospy.logerr(f"Failed to read PGM file: {pgm_file_path}")
            return None

        # Get image dimensions
        height, width = map_image.shape
        
        # Calculate center coordinates
        center_x = width // 2
        center_y = height // 2
        
        # Extract 400x400 pixels from center
        crop_size = 300
        half_crop = crop_size // 2
        
        # Calculate crop boundaries
        x1 = max(0, center_x - half_crop)
        y1 = max(0, center_y - half_crop)
        x2 = min(width, center_x + half_crop)
        y2 = min(height, center_y + half_crop)
        
        # Crop the center region
        cropped_image = map_image[y1:y2, x1:x2]
        
        # Convert to PIL Image
        pil_image = Image.fromarray(cropped_image, mode='L')
        
        # Save as PNG to preserve quality (lossless)
        buffered = io.BytesIO()
        pil_image.save(buffered, format="PNG")
        img_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')

        rospy.loginfo(f"Extracted center 400x400 region from PGM file {pgm_file_path} and converted to base64 successfully.")
        return img_base64

    except Exception as e:
        rospy.logerr(f"Exception occurred while converting PGM to base64: {e}")
        return None


class TaskManagerNode:
    # TM自身管理的状态字符串常量 (这些会通过RobotStatusCompressed发送给WSRB->Rails)
    STATE_OFFLINE = "offline" # 初始或连接断开
    STATE_INITIALIZING = "initializing"
    STATE_IDLE = "idle"
    STATE_MAPPING_AUTO = "mapping_auto" # 自动建图任务
    STATE_NAVIGATING_TASK = "navigating" # 执行导航任务
    STATE_MANUAL_CONTROL = "manual_control" # 手动控制模式
    STATE_EXECUTING_TASK = "executing_task" # 通用任务执行状态
    # STATE_CANCELLING_TASK = "cancelling_task" # 取消任务中
    STATE_EMERGENCY_STOPPED = "emergency_stopped"
    STATE_FETCHING_BOOK = "fetching_book" # 取书任务
    STATE_SCANNING = "scanning" # 扫描任务
    STATE_ERROR = "error"

    def _get_state_for_task_type(self, task_type):
        state_mapping = {
            "MAP_BUILD_AUTO": self.STATE_MAPPING_AUTO,
            # "COMPLETE_MAP_BUILD": self.STATE_EXECUTING_TASK,
            "NAVIGATION_TO_POINT": self.STATE_NAVIGATING_TASK,
            "LOAD_MAP": self.STATE_EXECUTING_TASK,
            "FETCH_BOOK_TO_TRANSFER": self.STATE_FETCHING_BOOK,
        }
        return state_mapping.get(task_type, self.STATE_EXECUTING_TASK)

    def __init__(self):
        # 注意：当通过launch文件启动时，不要调用rospy.init_node()
        # rospy.init_node('task_manager_node', anonymous=True)
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing TaskManager...")

        # 从ROS参数服务器获取TF框架参数
        global MAP_FRAME_ID, ODOM_FRAME_ID, BASE_FRAME_ID
        MAP_FRAME_ID = rospy.get_param("~map_frame", "map")
        ODOM_FRAME_ID = rospy.get_param("~odom_frame", "odom")
        BASE_FRAME_ID = rospy.get_param("~base_frame", "base_link")

        # 内部状态变量
        self.current_tm_state = self.STATE_INITIALIZING # TM自身的状态
        self.current_error_msg = None
        self.is_robot_emergency_stopped = False # 独立的急停标志
        self.active_map_in_ros = rospy.get_param("~initial_active_map", 0) # 当前激活的地图ID

        # 机器人实时数据 (由ROS回调更新)
        self.pose_data = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.velocity_data = {"linear": 0.0, "angular": 0.0}
        self.battery_percentage = 99.0 # 模拟
        self.has_global_pose = False # 是否从map frame获取到TF

        self.data_lock = threading.RLock()

        # 任务调度系统
        self.task_priority_queue = []
        self.current_executing_task_info = None
        self.task_scheduler_lock = threading.RLock()

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_update_thread = threading.Thread(target=self._tf_update_loop)
        self.tf_update_thread.daemon = True
        self.tf_update_thread.start()

        # ROS Publishers & Subscribers
        rospy.Subscriber(TM_DIRECTIVE_TOPIC, TaskDirective, self._directive_callback, queue_size=20)
        self.task_feedback_pub = rospy.Publisher(TM_FEEDBACK_TOPIC, TaskFeedback, queue_size=20)
        self.robot_status_compressed_pub = rospy.Publisher(ROBOT_STATUS_TOPIC, RobotStatusCompressed, queue_size=10, latch=True)
        self.cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)

        rospy.Subscriber(ODOM_TOPIC, Odometry, self._odom_callback, queue_size=5)
        # rospy.Subscriber(AMCL_POSE_TOPIC, PoseWithCovarianceStamped, self._amcl_pose_callback, queue_size=5) # 如果使用

        # 定期发布 RobotStatusCompressed
        self.status_publish_rate = rospy.get_param("~status_publish_rate_hz", 0.2)
        self.status_publish_timer = rospy.Timer(rospy.Duration(1.0 / self.status_publish_rate), self._publish_robot_status_event)
        
        # 任务处理主循环线程
        self.task_processor_thread = threading.Thread(target=self._task_processing_loop)
        self.task_processor_thread.daemon = True
        self.task_processor_thread.start()

        # 模拟电池消耗线程
        self.battery_sim_thread = threading.Thread(target=self._simulate_battery_drain_loop)
        self.battery_sim_thread.daemon = True
        self.battery_sim_thread.start()
        
        self._change_tm_state_and_publish(self.STATE_IDLE) # 初始化完成，进入空闲
        rospy.loginfo(f"[{self.node_name}] TaskManager Initialized and now IDLE.")

    def _change_tm_state_and_publish(self, new_state_str, error_message=None):
        # 调用此方法时不能持有 data_lock
        state_changed = False
        with self.data_lock:
            if self.current_tm_state != new_state_str:
                rospy.loginfo(f"[{self.node_name}] TM State: {self.current_tm_state} -> {new_state_str}")
                self.current_tm_state = new_state_str
                state_changed = True
            
            if self.current_error_msg != error_message: # Allow clearing error by passing None
                self.current_error_msg = error_message
                state_changed = True
        
        if state_changed:
            self._publish_robot_status() # 如果状态有任何重要变化，立即发布一次

    def _publish_robot_status_event(self, event=None): # Timer回调
        self._publish_robot_status()

    def _publish_robot_status(self): # 将当前TM的状态发布出去
        # with self.data_lock, self.task_scheduler_lock:
        msg = RobotStatusCompressed()
        msg.header.stamp = rospy.Time.now()
        msg.pose_x = float(self.pose_data.get("x", 0.0))
        msg.pose_y = float(self.pose_data.get("y", 0.0))
        msg.pose_theta = float(self.pose_data.get("theta", 0.0))
        msg.velocity_linear = float(self.velocity_data.get("linear", 0.0))
        msg.velocity_angular = float(self.velocity_data.get("angular", 0.0))
        msg.battery_percentage = float(self.battery_percentage)
        msg.robot_state_str = self.current_tm_state # TM自身管理的核心状态
        msg.error_message = self.current_error_msg or ""
        msg.is_emergency_stopped = self.is_robot_emergency_stopped
        msg.active_task_id_rails = int(self.current_executing_task_info.task_id or 0) if self.current_executing_task_info else 0
        msg.active_map = self.active_map_in_ros or 0
        self.robot_status_compressed_pub.publish(msg)
        rospy.logdebug(f"[{self.node_name}] Published RobotStatusCompressed: State='{msg.robot_state_str}', EStop={msg.is_emergency_stopped}")

    def _publish_task_feedback(self, rails_feedback_action_name, feedback_payload_dict):
        feedback_msg = TaskFeedback()
        feedback_msg.header.stamp = rospy.Time.now()
        feedback_msg.feedback_action_rails = rails_feedback_action_name
        feedback_msg.feedback_payload_json = json.dumps(feedback_payload_dict)
        self.task_feedback_pub.publish(feedback_msg)
        rospy.loginfo(f"[{self.node_name}] Sent TaskFeedback to WSRB: Action='{feedback_msg.feedback_action_rails}'")

    # --- ROS Callbacks for Robot Data ---
    def _odom_callback(self, msg):
        with self.data_lock:
            self.velocity_data["linear"] = msg.twist.twist.linear.x
            self.velocity_data["angular"] = msg.twist.twist.angular.z
            # 只有在没有全局位姿时才使用Odom的位姿（由TF loop控制has_global_pose）
            if not self.has_global_pose:
                self.pose_data["x"] = msg.pose.pose.position.x
                self.pose_data["y"] = msg.pose.pose.position.y
                q = msg.pose.pose.orientation
                try:
                    (_, _, self.pose_data["theta"]) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                except Exception as e:
                    rospy.logwarn_throttle(10, f"[{self.node_name}] Odom quat to euler failed: {e}")

    def _tf_update_loop(self): # 用于从TF获取全局位姿
        rate = rospy.Rate(rospy.get_param("~tf_update_rate_hz", 2.0)) # 默认2Hz
        while not rospy.is_shutdown():
            transform = None
            try:
                transform = self.tf_buffer.lookup_transform(MAP_FRAME_ID, BASE_FRAME_ID, rospy.Time(0), rospy.Duration(0.5))
                with self.data_lock:
                    self.pose_data["x"] = transform.transform.translation.x
                    self.pose_data["y"] = transform.transform.translation.y
                    q = transform.transform.rotation
                    (_, _, self.pose_data["theta"]) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                    self.has_global_pose = True
                rospy.logdebug_throttle(10, f"[{self.node_name}] TF Global Pose updated from '{MAP_FRAME_ID}'.")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e_map:
                rospy.logdebug_throttle(10, f"[{self.node_name}] TF: Cannot get '{MAP_FRAME_ID}'->'{BASE_FRAME_ID}': {e_map}. Trying Odom frame.")
                with self.data_lock: self.has_global_pose = False # 标记无法获取全局位姿
                # 如果map->base_link失败，可以尝试odom->base_link，但通常odom_callback已处理这个（如果has_global_pose=False）
            rate.sleep()
        rospy.loginfo(f"[{self.node_name}] TF updater loop terminated.")


    def _simulate_battery_drain_loop(self):
        rate = rospy.Rate(0.05) # 每20秒掉一点电
        while not rospy.is_shutdown():
            with self.data_lock:
                drain = 0.0
                if self.current_tm_state not in [self.STATE_IDLE, self.STATE_OFFLINE, self.STATE_EMERGENCY_STOPPED]:
                    drain = random.uniform(0.08, 0.15) # 工作时耗电多
                else:
                    drain = random.uniform(0.01, 0.03) # 空闲时耗电少
                self.battery_percentage = max(0, self.battery_percentage - drain)
            rate.sleep()

    # --- Directive Processing ---
    def _directive_callback(self, directive): # Type: TaskDirective
        rospy.loginfo(f"[{self.node_name}] Received Directive: Cmd='{directive.command_type}', TaskID='{directive.task_id_rails}'")
        
        # 优先级1: 急停 (可以中断任何事情)
        if directive.command_type == "EMERGENCY_STOP":
            self._execute_emergency_stop(directive)
            return

        # 优先级1: 取消任务 (可以中断任何任务)
        if directive.command_type == "CANCEL_TASK":
            self._request_cancel_task(directive.parameters_json.get("task_id", "0"))
            return

        with self.data_lock:
            # 优先级2: 如果当前是急停状态，特殊处理
            if self.is_robot_emergency_stopped:
                if directive.command_type == "RESUME_OPERATION":
                    self._execute_resume_from_estop_or_error(directive)
                elif directive.command_type == "ENABLE_MANUAL_CONTROL":
                    self._execute_enable_manual_control_from_estop(directive)
                else:
                    rospy.logwarn(f"[{self.node_name}] Cmd '{directive.command_type}' rejected: Robot is E-STOPPED. Only RESUME or ENABLE_MANUAL is allowed.")
                    self._publish_task_feedback("report_control_completion", {
                        "final_status_from_ros": "failed_robot_estopped",
                        "message": "TM: Command rejected, robot is emergency stopped."
                    })
                return

            # 优先级3: 如果当前是自动建图模式
            # TODO: 将该指令视为即时指令, 需要修改对应的处理
            if self.current_tm_state == self.STATE_MAPPING_AUTO:
                if directive.command_type == "COMPLETE_MAP_BUILD":
                    self._execute_complete_map_build(TaskInfo(directive)) # 直接执行建图完成
                else:
                    rospy.logwarn(f"[{self.node_name}] Cmd '{directive.command_type}' rejected: Robot in MAPPING_AUTO mode. Only COMPLETE_MAP_BUILD or CANCEL_TASK is allowed.")
                    self._publish_task_feedback("report_control_completion", {
                        "final_status_from_ros": "failed_robot_mapping_mode",
                        "message": "TM: Command rejected, robot is in mapping mode."
                    })
                return

            # 优先级4: 如果当前是手动控制模式
            if self.current_tm_state == self.STATE_MANUAL_CONTROL:
                if directive.command_type == "MOVE":
                    self._execute_manual_move(directive)
                elif directive.command_type == "DISABLE_MANUAL_CONTROL": # 新增指令，从手动模式退出到IDLE
                    self._execute_disable_manual_control(directive)
                else: # 手动模式下不接受任务指令
                    rospy.logwarn(f"[{self.node_name}] Cmd '{directive.command_type}' rejected: Robot in MANUAL_CONTROL mode. Only MOVE or DISABLE_MANUAL is allowed.")
                    if directive.task_id_rails and directive.task_id_rails != "0": # 如果是任务型指令
                        self._publish_task_feedback("report_task_completion", {
                            "task_id": int(directive.task_id_rails), 
                            "final_status_from_ros": "failed_robot_manual_mode",
                            "message": "TM: Command rejected, robot is in manual control mode."
                        })
                return
            
        # 优先级5: 任务型指令 
        if directive.command_type == "TASK_EXECUTE":
            self._add_task_to_priority_queue(directive)
        else:
            rospy.logwarn(f"[{self.node_name}] Unhandled command: {directive.command_type}")
    
    def _add_task_to_priority_queue(self, directive):
        if not directive.task_id_rails or directive.task_id_rails == "0":
            rospy.logwarn(f"[{self.node_name}] Invalid task_id_rails.")
            return
        
        task_info = TaskInfo(directive)

        with self.task_scheduler_lock:
            for existing_task in self.task_priority_queue:
                if existing_task.task_id == task_info.task_id:
                    rospy.logwarn(f"[{self.node_name}] Task {task_info.task_id} already in queue.")
                    return

            if self.current_executing_task_info and self.current_executing_task_info.task_id == task_info.task_id:
                rospy.logwarn(f"[{self.node_name}] Task {task_info.task_id} is currently executing")
                return

            queue_size = len(self.task_priority_queue)
            if queue_size >= 10:
                rospy.logwarn(f"[{self.node_name}] Task queue is full ({queue_size} tasks). Cannot add task {task_info.task_id}.")
                self._publish_task_feedback("report_task_completion", {
                    "task_id": int(task_info.task_id),
                    "final_status_from_ros": "failed_queue_full",
                    "message": f"TM: Task queue is full ({queue_size} tasks). Cannot add task {task_info.task_id}."
                })
                return
            
            heapq.heappush(self.task_priority_queue, task_info) # 使用堆保持优先级顺序
            rospy.loginfo(f"[{self.node_name}] Added task {task_info.task_id} (priority: {task_info.priority}) to queue. Size: {queue_size}.")

            self._publish_task_feedback("update_task_progress", {
                "task_id": int(task_info.task_id),
                "status_from_ros": "tm_task_queued",
                # "progress_percentage": 1,
                "message": f"TM: Task queued with priority {task_info.priority}",
                "result_data": {
                    "queue_position": queue_size + 1
                }
            })
    
    def _request_cancel_task(self, task_id):
        with self.task_scheduler_lock:
            need_cancel = True if self.current_executing_task_info and self.current_executing_task_info.task_id == task_id else False
        if need_cancel:
            rospy.loginfo(f"[{self.node_name}] Cancelling current task {task_id}.")
            self._publish_task_feedback("report_task_completion", {
                "task_id": int(task_id or 0),
                "final_status_from_ros": "cancelling_by_user",
                "message": f"TM: Task {task_id} cancelling by user."
            })
            self.current_executing_task_info.set_cancelled()
            self.current_executing_task_info = None
            # TODO: 结束当前任务（终止其他所有roslaunch进程）
            # func(bool: flag), 取消则直接丢弃，急停则要重新加入队列（优先级+1）
            # 发送cancelled反馈
            self._reboot_ros_for_task_cancellation("cancel_task") # 重新启动ROS以取消任务
            self._change_tm_state_and_publish(self.STATE_IDLE, "Task cancelled by user.")
            self._publish_task_feedback("report_task_completion", {
                "task_id": int(task_id or 0), 
                "final_status_from_ros": "cancelled_by_user",
                "message": f"TM: Task {task_id} cancelled by user."
            })
            return
        with self.data_lock:
            new_queue = []
            task_found = False
            for task in self.task_priority_queue:
                if task.task_id == task_id:
                    task_found = True
                    rospy.loginfo(f"[{self.node_name}] Task {task_id} cancelled from queue.")
                    self._publish_task_feedback("report_task_completion", {
                        "task_id": int(task_id or 0),
                        "final_status_from_ros": "cancelled_by_user",
                        "message": f"TM: Task {task_id} cancelled by user."
                    })
                else:
                    new_queue.append(task)
            
            if task_found:
                self.task_priority_queue = new_queue
                heapq.heapify(self.task_priority_queue) # 重新调整堆结构
            else:
                rospy.logwarn(f"[{self.node_name}] Task {task_id} not found in queue for cancellation.")
                self._publish_task_feedback("report_task_completion", {
                    "task_id": int(task_id or 0),
                    "final_status_from_ros": "failed_task_not_found",
                    "message": f"TM: Task {task_id} not found in queue for cancellation."
                })

    def _task_processing_loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            task_to_execute = None
            with self.task_scheduler_lock, self.data_lock:
                can_start_new_task = (
                    self.current_tm_state == self.STATE_IDLE and
                    not self.is_robot_emergency_stopped and
                    self.current_executing_task_info is None and
                    len(self.task_priority_queue) > 0
                )

                if can_start_new_task:
                    task_to_execute = heapq.heappop(self.task_priority_queue)
                    self.current_executing_task_info = task_to_execute
            
            if task_to_execute:
                rospy.loginfo(f"[{self.node_name}] Starting execution thread for Task {task_to_execute.task_id} ({task_to_execute.task_type})")
                task_thread = threading.Thread(
                    target=self._task_execution_wrapper, 
                    args=(task_to_execute,)
                )
                task_thread.daemon = True
                task_thread.start()
            rate.sleep()

    def _task_execution_wrapper(self, task_info):
        try:
            task_state = self._get_state_for_task_type(task_info.task_type)
            rospy.loginfo(f"[{self.node_name}] Task {task_info.task_id} is in state {task_state}.")
            self._change_tm_state_and_publish(task_state)
            self._dispatch_task_execution(task_info)

        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error occurred while executing task {task_info.task_id}: {e}")
            self._publish_task_feedback('report_task_completion', {
                "task_id": int(task_info.task_id or 0),
                "final_status_from_ros": "failed",
                "message": f"TM: Error in task execution: {str(e)}"
            })
        with self.task_scheduler_lock:
            if self.current_executing_task_info == task_info and not self.current_tm_state == self.STATE_MAPPING_AUTO:
                self.current_executing_task_info = None
        need_change_state = False
        with self.data_lock:
            if not self.is_robot_emergency_stopped and not self.current_tm_state == self.STATE_MAPPING_AUTO:
                need_change_state = True
        if need_change_state:
            self._change_tm_state_and_publish(self.STATE_IDLE)

    def _dispatch_task_execution(self, task_info):
        task_id = task_info.task_id
        task_type = task_info.task_type

        self._publish_task_feedback("update_task_progress", {
            "task_id": int(task_id or 0),
            "status_from_ros": "tm_task_started",
            # "progress_percentage": 10,
            "message": f"TM: Task {task_id} started execution."
        })

        if task_type == "MAP_BUILD_AUTO":
            self._execute_map_build_auto(task_info)
        elif task_type == "LOAD_MAP":
            self._execute_load_map(task_info)
        elif task_type == "NAVIGATION_TO_POINT":
            self._execute_navigation_to_point(task_info)
        elif task_type == "FETCH_BOOK_TO_TRANSFER":
            self._execute_fetch_book_to_transfer(task_info)
        else:
            rospy.logwarn(f"[{self.node_name}] Unknown task type: {task_type}")

    def _execute_emergency_stop(self, directive):
        with self.data_lock:
            rospy.logfatal(f"[{self.node_name}] EXECUTING EMERGENCY STOP!")
            self.is_robot_emergency_stopped = True

            self.cmd_vel_pub.publish(Twist()) # 停止所有运动
            self._reboot_ros_for_task_cancellation("emergency_stop") # 重新启动ROS以取消任务
        
        self._change_tm_state_and_publish(
            self.STATE_EMERGENCY_STOPPED,
            "Robot emergency stopped. Current task cancelled."
        )

        with self.task_scheduler_lock:
            if self.current_executing_task_info:
                rospy.loginfo(f"[{self.node_name}] Cancelling current task {self.current_executing_task_info.task_id} due to emergency stop.")
                self.current_executing_task_info.set_cancelled()
                self._publish_task_feedback("update_task_progress", {
                    "task_id": int(self.current_executing_task_info.task_id or 0),
                    "final_status_from_ros": "paused_by_emergency_stop",
                    "message": f"TM: Task {self.current_executing_task_info.task_id} paused due to emergency stop."
                })

                self.current_executing_task_info.priority += 1 # 提升优先级以便重新调度
                self.task_priority_queue.append(self.current_executing_task_info)
                heapq.heapify(self.task_priority_queue) # 重新调整堆结构
                self.current_executing_task_info = None

    def _reboot_ros_for_task_cancellation(self, type): # 手动取消 or 急停取消正在执行的任务
        rospy.loginfo(f"[{self.node_name}] Rebooting ROS for ({type})...")    
        def call_service_safely(service_name, service_type, request_args=[]):
            """安全调用ROS服务"""
            try:
                rospy.wait_for_service(service_name, timeout=2.0)
                service_proxy = rospy.ServiceProxy(service_name, service_type)
                
                if request_args:
                    response = service_proxy(**request_args)
                else:
                    response = service_proxy()
                if response.success:
                    rospy.loginfo(f"[{self.node_name}] Service {service_name} called successfully")
                else:
                    rospy.logwarn(f"[{self.node_name}] Service {service_name} returned failure: {response.message}")
                    return False
                return True
                
            except rospy.ServiceException as e:
                rospy.logwarn(f"[{self.node_name}] Service {service_name} failed: {e}")
                return False
            except rospy.ROSException as e:
                rospy.logwarn(f"[{self.node_name}] Service {service_name} not available: {e}")
                return False
            except Exception as e:
                rospy.logwarn(f"[{self.node_name}] Unexpected error calling {service_name}: {e}")
                return False
        
        def kill_node_safely(node_name):
            """安全杀死ROS节点"""
            try:
                result = subprocess.run(['rosnode', 'kill', node_name], 
                                    capture_output=True, text=True, timeout=3.0)
                if result.returncode == 0:
                    rospy.loginfo(f"[{self.node_name}] Node {node_name} killed")
                    return True
                else:
                    rospy.logwarn(f"[{self.node_name}] Failed to kill {node_name}")
                    return False
            except Exception as e:
                rospy.logwarn(f"[{self.node_name}] Error killing {node_name}: {e}")
                return False
        
        def start_node_safely(package, executable):
            """安全启动ROS节点"""
            try:
                subprocess.Popen(['rosrun', package, executable])
                rospy.loginfo(f"[{self.node_name}] Started {package}/{executable}")
                return True
            except Exception as e:
                rospy.logwarn(f"[{self.node_name}] Error starting {package}/{executable}: {e}")
                return False
        
        try:
            # Phase 1: 停止命令
            rospy.loginfo(f"[{self.node_name}] Phase 1: Executing halt commands...")

            call_service_safely('/arm_emergency_stop', Empty)
            
            kill_node_safely('/wpb_home_grab_action')
            
            call_service_safely('/emergency_stop', Empty)
            
            call_service_safely('/halt_mapping', Halt, {'path': './maps/', 'name': 'temp'})
            time.sleep(1.0)
            
            
            # Phase 2: 恢复命令
            rospy.loginfo(f"[{self.node_name}] Phase 2: Executing restore commands...")

            call_service_safely('/arm_zero_service', Trigger)
            
            start_node_safely('wpb_home_behaviors', 'wpb_home_grab_action')
            
            rospy.loginfo(f"[{self.node_name}] ROS reboot sequence completed for ({type})")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Unexpected error during ROS reboot for ({type}): {e}")
        # TODO: 是否还需要重新启动其他节点或服务？比如导航、建图等？

    def _execute_resume_from_estop_or_error(self, directive):
        with self.data_lock:
            if not self.is_robot_emergency_stopped:
                rospy.logwarn(f"[{self.node_name}] RESUME_OPERATION rejected: Robot not in E-STOPPED state.")
                return
            
            rospy.loginfo(f"[{self.node_name}] Resuming operation from E-STOPPED state...")
            self.is_robot_emergency_stopped = False

        self._change_tm_state_and_publish(
            self.STATE_IDLE, 
            "Robot resumed from E-STOPPED state."
        )
        rospy.loginfo(f"[{self.node_name}] Robot now IDLE.")
        self._publish_task_feedback("report_control_completion", {
            "final_status_from_ros": "success",
            "message": "TM: Robot resumed from E-STOPPED state."
        })

    def _execute_enable_manual_control_from_estop(self, directive):
        with self.data_lock:
            if not self.is_robot_emergency_stopped:
                rospy.logwarn(f"[{self.node_name}] ENABLE_MANUAL_CONTROL rejected: Robot not in E-STOPPED state.")
                return
            
            rospy.loginfo(f"[{self.node_name}] Enabling MANUAL_CONTROL mode from E-STOPPED state...")
            self.is_robot_emergency_stopped = False

        self._change_tm_state_and_publish(
            self.STATE_MANUAL_CONTROL, 
            "Manual control mode enabled."
        )
        rospy.loginfo(f"[{self.node_name}] Robot now in MANUAL_CONTROL mode.")
        self._publish_task_feedback("report_control_completion", {
            "final_status_from_ros": "success",
            "message": "TM: Manual control mode enabled from E-STOPPED state."
        })

    def _execute_disable_manual_control(self, directive):
        with self.data_lock:
            if self.current_tm_state != self.STATE_MANUAL_CONTROL:
                rospy.logwarn(f"[{self.node_name}] DISABLE_MANUAL_CONTROL rejected: Robot not in MANUAL_CONTROL mode.")
                return
            
            rospy.loginfo(f"[{self.node_name}] Disabling MANUAL_CONTROL mode, returning to E-STOPPED state...")
            self.is_robot_emergency_stopped = True

        self._change_tm_state_and_publish(
            self.STATE_EMERGENCY_STOPPED,
            "Manual control mode disabled, returning to E-STOPPED state."
        )
        self.cmd_vel_pub.publish(Twist())
        self._publish_task_feedback("report_control_completion", {
            "final_status_from_ros": "success",
            "message": "TM: Manual control mode disabled, robot stopped."
        })

    def _execute_manual_move(self, directive):
        with self.data_lock:
            if self.current_tm_state != self.STATE_MANUAL_CONTROL:
                rospy.logwarn(f"[{self.node_name}] MOVE command rejected: Robot not in MANUAL_CONTROL mode (current: {self.current_tm_state}).")
                return
        
        try:
            params = json.loads(directive.parameters_json) if directive.parameters_json else {}
            direction = params.get("direction", "stop")
            speed = float(params.get("speed", 0.0))
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Invalid parameters for MOVE command: {directive.parameters_json}. Error: {e}")
            return

        twist_msg = Twist()
        if direction == "forward": twist_msg.linear.x = speed
        elif direction == "backward": twist_msg.linear.x = -speed
        elif direction == "left": twist_msg.angular.z = speed
        elif direction == "right": twist_msg.angular.z = -speed
        elif direction == "stop": pass # Default Twist is zero
        else: rospy.logwarn(f"[{self.node_name}] Unknown move direction: {direction}"); return
        
        self.cmd_vel_pub.publish(twist_msg)
        rospy.loginfo(f"[{self.node_name}] MANUAL_CONTROL: Published Twist: lin.x={twist_msg.linear.x:.2f}, ang.z={twist_msg.angular.z:.2f}")
        self._publish_task_feedback("report_control_completion", {
            "final_status_from_ros": "success",
            "message": f"TM: Manual move command executed: {direction} at speed {speed:.2f}."
        })

    # --- Specific Task Execution Methods ---

    def _execute_map_build_auto(self, task_info):
        try:
            params = task_info.params
            task_id = task_info.task_id
            rospy.loginfo(f"[{self.node_name}] Starting auto mapping for task {task_id} with params: {params}")
            sim = params.get("sim", True)

            rospy.wait_for_service(AUTO_MAPPING_SERVICE_NAME, timeout=5.0)
            auto_mapping = rospy.ServiceProxy(AUTO_MAPPING_SERVICE_NAME, Start)
            res = auto_mapping(sim=sim)

            if res.success:
                rospy.loginfo(f"[{self.node_name}] Auto mapping started successfully.")
                self._publish_task_feedback("report_task_completion", {
                    "task_id": int(task_id or 0),
                    "final_status_from_ros": "success",
                    "message": "TM: Auto mapping started successfully."
                })
            else:
                rospy.logerr(f"[{self.node_name}] Auto mapping failed: {res.message}")
                self._publish_task_feedback("report_task_completion", {
                    "task_id": int(task_id or 0),
                    "final_status_from_ros": "failed",
                    "message": f"TM: Auto mapping failed: {res.message}"
                })
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Exception occurred while starting auto mapping: {e}")
            self._publish_task_feedback("report_task_completion", {
                "task_id": int(task_id or 0),
                "final_status_from_ros": "failed",
                "message": f"TM: Exception in auto mapping: {str(e)}"
            })

    def _execute_complete_map_build(self, task_info): # 这其实是即时指令，不是任务型指令
        with self.data_lock: # 检查当前状态是否允许完成建图
            if self.current_tm_state != self.STATE_MAPPING_AUTO:
                rospy.logwarn(f"[{self.node_name}] COMPLETE_MAP_BUILD command rejected: Robot not in MAPPING_AUTO state (current: {self.current_tm_state}).")
                return
        try:
            params = task_info.params
            rospy.loginfo(f"[{self.node_name}] Completing map build for task {self.current_executing_task_info.task_id} with params: {params}")
            map_id = params.get("map_id")
            map_name = params.get("map_name")

            rospy.wait_for_service(HALT_MAPPING_SERVICE_NAME, timeout=5.0)
            halt_mapping = rospy.ServiceProxy(HALT_MAPPING_SERVICE_NAME, Halt)
            res = halt_mapping(path="./", name=map_name)

            if res.success:
                rospy.loginfo(f"[{self.node_name}] Map build completed successfully.")

                map_image_base64 = convert_pgm_to_image_base64("/media/mitchell/Data/Projects/software_engineering/ros_end/src/mapping/" + "./maps/" + map_name + ".pgm")

                if not map_image_base64:
                    rospy.logwarn(f"[{self.node_name}] Failed to convert map image to base64.")
                    map_image_base64 = ""

                self._publish_task_feedback("report_control_completion", {
                    "type": "complete_map_build",
                    "final_status_from_ros": "success",
                    "message": f"TM: Map build completed successfully. Map name: {map_name}",
                    "result_data": {
                        "map_id": map_id,
                        "map_data_url": "./",  # 暂定所有地图都存放在/Maps目录下
                        "map_image": map_image_base64
                    }
                })
            else:
                rospy.logerr(f"[{self.node_name}] Map build completion failed: {res.message}")
                self._publish_task_feedback("report_control_completion", {
                    "type": "complete_map_build",
                    "final_status_from_ros": "failed",
                    "message": f"TM: Map build completion failed: {res.message}",
                    "result_data": {
                        "map_id": map_id,
                    }
                })
            self._change_tm_state_and_publish(self.STATE_IDLE, "Map build completed.")
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Exception occurred while completing map build: {e}")
            self._publish_task_feedback("report_control_completion", {
                "type": "complete_map_build",
                "final_status_from_ros": "failed",
                "message": f"TM: Exception in completing map build: {str(e)}",
                "result_data": {
                    "map_id": params.get("map_id"),
                }
            })

    def _execute_load_map(self, task_info):
        params = task_info.params
        task_id = task_info.task_id
        try:
            map_id = params.get("map_id")
            map_name = params.get("map_name")
            map_data_url = params.get("map_data_url")
            rospy.loginfo(f"[{self.node_name}] Loading map '{map_data_url + map_name}' for task {task_id}")

            rospy.wait_for_service(NAVIGATION_ENABLE_SERVICE_NAME, timeout=5.0)
            load_map = rospy.ServiceProxy(NAVIGATION_ENABLE_SERVICE_NAME, nStart)
            res = load_map(sim=True, map=True, path=map_data_url, name=map_name)

            if res.success:
                rospy.loginfo(f"[{self.node_name}] Map '{map_data_url + map_name}' loaded successfully.")
                self._publish_task_feedback("report_task_completion", {
                    "task_id": int(task_id or 0),
                    "final_status_from_ros": "success",
                    "message": f"TM: Map '{map_data_url + map_name}' loaded successfully.",
                    "result_data": {
                        "map_id": map_id,
                    }
                })
            else:
                rospy.logerr(f"[{self.node_name}] Map loading failed: {res.message}")
                self._publish_task_feedback("report_task_completion", {
                    "task_id": int(task_id or 0),
                    "final_status_from_ros": "failed",
                    "message": f"TM: Map loading failed: {res.message}"
                })
            with self.data_lock:
                self.active_map_in_ros = map_id # 更新当前激活的地图
                self.has_global_pose = True # 成功加载地图后，认为有全局位姿
            # self._change_tm_state_and_publish(self.STATE_IDLE, "Map loaded successfully.")
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Exception occurred while loading map: {e}")
            self._publish_task_feedback("report_task_completion", {
                "task_id": int(task_id or 0),
                "final_status_from_ros": "failed",
                "message": f"TM: Exception in loading map: {str(e)}"
            })

    def _execute_navigation_to_point(self, task_info):
        with self.data_lock:
            if self.active_map_in_ros is None:
                rospy.logwarn(f"[{self.node_name}] NAVIGATION_TO_POINT command rejected: No active map loaded.")
                return
            rospy.loginfo(f"[{self.node_name}] NAVIGATION_TO_POINT command accepted with active map: {self.active_map_in_ros}")
        try:
            params = task_info.params
            task_id = task_info.task_id
            px = params.get("px")
            py = params.get("py")
            oz = params.get("oz")
            rospy.loginfo(f"[{self.node_name}] Navigating to pose ({px}, {py}, {oz}) for task {task_id}")

            rospy.wait_for_service(NAVIGATION_GOAL_SERVICE_NAME, timeout=5.0)
            navigate_to_pose = rospy.ServiceProxy(NAVIGATION_GOAL_SERVICE_NAME, Goal)
            res = navigate_to_pose(px=px, py=py, oz=oz)

            if res.success:
                rospy.loginfo(f"[{self.node_name}] Navigation to pose ({px}, {py}, {oz}) successful.")
                self._publish_task_feedback("report_task_completion", {
                    "task_id": int(task_id or 0),
                    "final_status_from_ros": "success",
                    "message": f"TM: Navigation to pose ({px}, {py}, {oz}) successful."
                })
            else:
                rospy.logerr(f"[{self.node_name}] Navigation to pose failed: {res.message}")
                self._publish_task_feedback("report_task_completion", {
                    "task_id": int(task_id or 0),
                    "final_status_from_ros": "failed",
                    "message": f"TM: Navigation to pose failed: {res.message}"
                })
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Exception occurred while navigating to pose: {e}")
            self._publish_task_feedback("report_task_completion", {
                "task_id": int(task_id or 0),
                "final_status_from_ros": "failed",
                "message": f"TM: Exception in navigating to pose: {str(e)}"
            })

    def _execute_fetch_book_to_transfer(self, task_info):
        with self.data_lock:
            if self.active_map_in_ros is None:
                rospy.logwarn(f"[{self.node_name}] FETCH_BOOK_TO_TRANSFER command rejected: No active map loaded.")
                return
            rospy.loginfo(f"[{self.node_name}] FETCH_BOOK_TO_TRANSFER command accepted with active map: {self.active_map_in_ros}")
        try:
            params = task_info.params
            task_id = task_info.task_id
            book_id = params.get("book_id")
            gpx = params.get("gpx")
            gpy = params.get("gpy")
            gpz = params.get("gpz")
            goz = params.get("goz")
            ppx = params.get("ppx")
            ppy = params.get("ppy")
            ppz = params.get("ppz")
            poz = params.get("poz")
            rospy.loginfo(f"[{self.node_name}] Fetching book {book_id} from ({gpx}, {gpy}, {gpz}, {goz}) to ({ppx}, {ppy}, {ppz}, {poz}) for task {task_id}")

            rospy.wait_for_service(FETCH_BOOK_SERVICE_NAME, timeout=5.0)
            fetch_book = rospy.ServiceProxy(FETCH_BOOK_SERVICE_NAME, Fetch)
            res = fetch_book(gpx=gpx, gpy=gpy, gpz=gpz, goz=goz, 
                ppx=ppx, ppy=ppy, ppz=ppz, poz=poz)
            
            if res.success:
                rospy.loginfo(f"[{self.node_name}] Fetch book {book_id} successful.")
                self._publish_task_feedback("report_task_completion", {
                    "task_id": int(task_id or 0),
                    "final_status_from_ros": "success",
                    "message": f"TM: Fetch book {book_id} successful.",
                    "result_data": {
                        "book_id": book_id,
                    }
                    # 可以增加抓取书籍的 isbn 识别结果, 这里假设书籍放置信息正确
                })
            else:
                rospy.logerr(f"[{self.node_name}] Fetch book failed: {res.message}")
                self._publish_task_feedback("report_task_completion", {
                    "task_id": int(task_id or 0),
                    "final_status_from_ros": "failed",
                    "message": f"TM: Fetch book failed: {res.message}"
                })
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Exception occurred while fetching book: {e}")
            self._publish_task_feedback("report_task_completion", {
                "task_id": int(task_id or 0),
                "final_status_from_ros": "failed",
                "message": f"TM: Exception in fetching book: {str(e)}"
            })

if __name__ == '__main__':
    tm_node_instance = None
    try:
        # 当作为独立脚本运行时初始化节点
        rospy.init_node('task_manager_node', anonymous=False)
        tm_node_instance = TaskManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"[{tm_node_instance.node_name if tm_node_instance and hasattr(tm_node_instance, 'node_name') else 'task_manager'}] Shutting down.")
    finally:
        node_name_for_log = tm_node_instance.node_name if tm_node_instance and hasattr(tm_node_instance, 'node_name') else 'task_manager'
        rospy.loginfo(f"[{node_name_for_log}] Performing final cleanup.")
        if tm_node_instance and hasattr(tm_node_instance, 'current_task_stop_event') and tm_node_instance.current_task_stop_event:
            rospy.loginfo(f"[{node_name_for_log}] Signaling stop for any active task thread on shutdown.")
            tm_node_instance.current_task_stop_event.set()
        rospy.loginfo(f"[{node_name_for_log}] Cleanup complete. Exiting.")