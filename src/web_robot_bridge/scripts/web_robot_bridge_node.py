#!/usr/bin/env python3
import rospy
import websocket
import threading
import json
import time
import tf2_ros
import queue
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import logging # 导入 logging 模块
import random # 导入 random 模块

# 设置 Python logging 模块的级别，确保 rospy.logdebug 能输出
logger = logging.getLogger('rosout')
logger.setLevel(logging.DEBUG)

# 本地测试为localhost
RAILS_APP_DOMAIN = "robot.einniemand.top"
# RAILS_APP_DOMAIN = "localhost"  # 本地测试
WEB_SERVER_URL = f"https://{RAILS_APP_DOMAIN}"
# 添加API密钥到WebSocket URL
WEBSOCKET_URL = f"wss://{RAILS_APP_DOMAIN}/cable" # 本地测试使用ws而非wss, 使用http, 并且需要指明端口为3000
API_KEY = "7ad0bbbdf00c5cbe87799355200f212ed329030028fd3ccd51524e461adf2c31"

STATUS_CHANNEL = "RobotStatusChannel"
COMMAND_CHANNEL = "RobotCommandChannel"

ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"
POSE_TOPIC = "/amcl_pose"  # AMCL或其他pose estimation发布的话题

class WebRobotBridgeNode:
    def __init__(self):
        """初始化ROS节点和必要的变量"""
        # 设置日志级别 (保留 rospy 参数设置，但上面已强制 Python logger)
        self.debug_mode = rospy.get_param('~debug_mode', True)
        rospy.set_param('~log_level', rospy.DEBUG) 
        rospy.loginfo(f"ROS Log Level set via param: {rospy.get_param('~log_level', 'Not Set')}")
        rospy.loginfo(f"Python Logger Level for 'rosout': {logging.getLogger('rosout').getEffectiveLevel()}")

        # 创建线程锁 (使用 RLock 解决重入问题)
        self.data_lock = threading.RLock()
        
        # 机器人数据
        self.position_x = 0.0
        self.position_y = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.battery_level = 100.0  # 模拟电池电量
        
        # 数据变化阈值和上次发送的位置
        self.last_sent_x = 0.0
        self.last_sent_y = 0.0
        self.last_sent_linear = 0.0
        self.last_sent_angular = 0.0
        self.position_threshold = 0.001  # 暂时设为 1 毫米
        self.velocity_threshold = 0.001  # 暂时设为 0.001 m/s 或 rad/s
        
        # 位置数据来源的优先级标志
        self.using_pose_estimate = False
        
        # 创建数据队列
        self.data_queue = queue.Queue(maxsize=10)
        
        # 创建TF缓存和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 创建发布者 - 发布到cmd_vel话题控制机器人
        self.cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        
        # 订阅里程计数据，作为位置数据的备份来源
        rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_callback)
        rospy.logdebug(f"Subscribed to {ODOM_TOPIC}") # 确认订阅
        
        # 订阅pose estimation话题
        rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback)
        rospy.logdebug(f"Subscribed to {POSE_TOPIC}") # 确认订阅
        
        # WebSocket连接对象
        self.ws = None
        self.ws_connected = False
        
        # 电池模拟参数
        self.last_sent_battery = self.battery_level
        self.battery_threshold = 1.0 # 电池电量变化超过 1% 时发送更新
        
        # 启动WebSocket连接管理线程
        self.ws_thread = threading.Thread(target=self.run_websocket)
        self.ws_thread.daemon = True
        self.ws_thread.start()
        
        # 启动位置数据发送线程
        self.position_thread = threading.Thread(target=self.send_position_data)
        self.position_thread.daemon = True
        self.position_thread.start()
        
        # 启动TF更新线程
        self.tf_thread = threading.Thread(target=self.update_position_from_tf)
        self.tf_thread.daemon = True
        self.tf_thread.start()
        
        # 启动心跳线程
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        
        # 启动电池模拟线程
        self.battery_thread = threading.Thread(target=self.simulate_battery_drain)
        self.battery_thread.daemon = True
        self.battery_thread.start()
        
        rospy.loginfo("Web Robot Bridge 节点启动成功！(包含电池模拟)") # 更新日志

    def connect_websocket(self):
        """创建WebSocket连接对象，但不启动连接"""
        try:
            # 根据debug模式决定是否启用trace
            websocket.enableTrace(self.debug_mode)
            
            # 添加自定义headers
            headers = {
                "X-Robot-API-Key": API_KEY
            }
            
            self.ws = websocket.WebSocketApp(
                # WEBSOCKET_URL + "?api_key=" + API_KEY,
                WEBSOCKET_URL,
                header=headers,
                on_message=lambda ws, msg: self.on_message(ws, msg),
                on_error=lambda ws, err: self.on_error(ws, err),
                on_close=lambda ws, close_status_code, close_msg: self.on_close(ws, close_status_code, close_msg),
                on_open=lambda ws: self.on_open(ws)
            )
            rospy.loginfo("WebSocket对象已创建")
        except Exception as e:
            rospy.logerr(f"创建WebSocket对象失败: {e}")
            self.ws = None

    def run_websocket(self):
        """在单一线程中管理WebSocket的连接和重连"""
        retry_interval = 5  # 重试间隔(秒)
        
        while not rospy.is_shutdown():
            try:
                if self.ws is None:
                    self.connect_websocket()
                
                if self.ws:
                    rospy.loginfo("启动WebSocket连接...")
                    self.ws_connected = False
                    self.ws.run_forever()
                    # run_forever退出后，表示连接已关闭
                    self.ws_connected = False
                    rospy.logwarn("WebSocket连接已断开")
                
                # 连接断开后等待一段时间再重试
                rospy.loginfo(f"将在{retry_interval}秒后重新连接...")
                time.sleep(retry_interval)
            except Exception as e:
                rospy.logerr(f"WebSocket运行错误: {e}")
                self.ws_connected = False
                time.sleep(retry_interval)
    
    def update_position_from_tf(self):
        """从TF树中获取最新的位置信息，优先级最高"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            try:
                # 尝试从map到base_link获取变换
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                
                with self.data_lock:
                    self.position_x = transform.transform.translation.x
                    self.position_y = transform.transform.translation.y
                    self.using_pose_estimate = True
                
                self.check_position_change()
                rospy.logdebug(f"从TF更新位置: x={self.position_x:.2f}, y={self.position_y:.2f}")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                try:
                    # 如果map到base_link不可用，尝试odom到base_link
                    transform = self.tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(0.1))
                    
                    with self.data_lock:
                        self.position_x = transform.transform.translation.x
                        self.position_y = transform.transform.translation.y
                        self.using_pose_estimate = False
                    
                    self.check_position_change()
                    rospy.logdebug(f"从odom到base_link更新位置: x={self.position_x:.2f}, y={self.position_y:.2f}")
                except:
                    rospy.logdebug_throttle(10, f"无法从TF获取位置: {e}")
            
            rate.sleep()
    
    def pose_callback(self, msg):
        """处理pose estimation话题的回调"""
        # rospy.loginfo("***** POSE CALLBACK ENTERED *****")
        # rospy.logdebug("--- pose_callback triggered ---")
        try:
            # rospy.loginfo("Pose Callback: Attempting to acquire data_lock...") # 新增日志
            with self.data_lock:
                # rospy.loginfo("Pose Callback: Acquired data_lock.")
                # rospy.loginfo("Pose Callback: Forcing update from pose data.")
                self.position_x = msg.pose.pose.position.x
                self.position_y = msg.pose.pose.position.y
                self.using_pose_estimate = True
                # rospy.loginfo(f"Pose Callback: Updated position to x={self.position_x:.2f}, y={self.position_y:.2f}, using_pose={self.using_pose_estimate}")
                # rospy.loginfo("Pose Callback: About to call check_position_change...") # 新增日志
                self.check_position_change()
                # rospy.loginfo("Pose Callback: Returned from check_position_change.") # 新增日志
                # else: 
                #    rospy.loginfo("Pose Callback: Skipped update, already using pose estimate? (using_pose_before={using_pose_before})")
            # rospy.loginfo("Pose Callback: Released data_lock.") # 新增日志
        except Exception as e:
            rospy.logerr(f"处理pose estimation数据出错: {e}")
    
    def on_open(self, ws):
        """WebSocket连接成功时的回调"""
        self.ws_connected = True
        rospy.loginfo("WebSocket连接成功")
        
        # 发送订阅命令通道消息
        subscription = {
            "command": "subscribe",
            "identifier": json.dumps({"channel": COMMAND_CHANNEL}),
        }
        try:
            ws.send(json.dumps(subscription))
            rospy.loginfo("已订阅RobotCommandChannel: %s", json.dumps(subscription))
        except Exception as e:
            rospy.logerr(f"订阅频道出错: {e}")
        
        # 添加订阅状态通道的代码
        status_subscription = {
            "command": "subscribe",
            "identifier": json.dumps({"channel": STATUS_CHANNEL}),
        }
        try:
            ws.send(json.dumps(status_subscription))
            rospy.loginfo("已订阅RobotStatusChannel: %s", json.dumps(status_subscription))
        except Exception as e:
            rospy.logerr(f"订阅状态频道出错: {e}")
    
    def on_message(self, ws, message):
        """处理从WebSocket接收到的消息"""
        try:
            rospy.loginfo(f"收到WebSocket消息: {message}") # 改为 INFO 方便查看
            data = json.loads(message)
            
            # 确认消息处理 (Welcome, Confirm Subscription, Ping)
            if 'type' in data:
                if data['type'] == 'welcome':
                    rospy.loginfo("收到WebSocket欢迎消息")
                elif data['type'] == 'confirm_subscription':
                    rospy.loginfo("订阅确认: %s", data)
                elif data['type'] == 'ping':
                    if self.debug_mode:
                        rospy.loginfo("收到ping")
                else:
                    rospy.logwarn(f"未知消息类型: {data['type']}")
                return
            
            # 处理命令消息
            identifier_data = data.get('identifier')
            if identifier_data:
                try:
                    identifier_json = json.loads(identifier_data)
                    if identifier_json.get('channel') != COMMAND_CHANNEL:
                         rospy.logwarn(f"收到非命令频道的消息，已忽略: {identifier_data}")
                         return # 忽略非来自命令频道的消息
                except json.JSONDecodeError:
                     rospy.logwarn(f"无法解析消息 identifier: {identifier_data}")
                     return
            else:
                 rospy.logwarn(f"收到缺少 identifier 的消息: {data}")
                 return # 忽略没有 identifier 的消息

            if 'message' in data and isinstance(data['message'], dict):
                cmd_data = data['message']
                cmd_type = cmd_data.get('command')
                
                rospy.loginfo(f"[Command Received] 解析到命令: type={cmd_type}, data={cmd_data}") # 添加日志

                if cmd_type == 'move':
                    # 从消息中提取速度值
                    direction = cmd_data.get('direction', 'stop')
                    speed = float(cmd_data.get('speed', 0.5))
                    
                    # 根据方向设置线速度和角速度
                    linear_x = 0.0
                    angular_z = 0.0
                    
                    if direction == 'forward':
                        linear_x = speed
                    elif direction == 'backward':
                        linear_x = -speed
                    elif direction == 'left':
                        angular_z = speed
                    elif direction == 'right':
                        angular_z = -speed
                    
                    # 创建并发布Twist消息
                    twist = Twist()
                    twist.linear.x = linear_x
                    twist.angular.z = angular_z
                    rospy.loginfo(f"[Command Execute] 发布 /cmd_vel: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}") # 添加日志
                    self.cmd_vel_pub.publish(twist)
                
                elif cmd_type == 'start_task':
                    # 处理任务命令
                    task_id = cmd_data.get('task_id')
                    task_type = cmd_data.get('task_type')
                    rospy.loginfo(f"收到任务命令: {task_type}, ID: {task_id}")
                    
                    # 这里可以实现任务逻辑，例如导航到特定位置
                    if task_type == "scan_shelves":
                        self.execute_shelf_scan_task()
                    elif task_type == "return_to_base":
                        self.execute_return_base_task()
            else:
                rospy.logwarn(f"[Command Received] 消息中缺少 'message' 字典: {data}")

        except Exception as e:
            rospy.logerr(f"处理WebSocket消息出错: {e}")
    
    def on_error(self, ws, error):
        """WebSocket错误处理"""
        rospy.logerr(f"WebSocket错误: {error}")
        self.ws_connected = False
    
    def on_close(self, ws, close_status_code, close_msg):
        """WebSocket连接关闭处理"""
        rospy.logwarn(f"WebSocket连接关闭: {close_msg}, 状态码: {close_status_code}")
        self.ws_connected = False
        # 不再在这里处理重连，由run_websocket线程管理重连
    
    def odom_callback(self, msg):
        """处理里程计数据"""
        # rospy.loginfo("***** ODOM CALLBACK ENTERED *****")
        # rospy.logdebug("--- odom_callback triggered ---")
        try:
            # rospy.loginfo("Odom Callback: Attempting to acquire data_lock...") # 新增日志
            with self.data_lock:
                # rospy.loginfo("Odom Callback: Acquired data_lock.")
                # 只有在没有更高优先级的位置源时才使用里程计位置数据
                if not self.using_pose_estimate:
                    # rospy.loginfo("Odom Callback: Updating position from odom.")
                    self.position_x = msg.pose.pose.position.x
                    self.position_y = msg.pose.pose.position.y
                    # rospy.loginfo(f"Odom Callback: Updated position to x={self.position_x:.2f}, y={self.position_y:.2f}")
                    # rospy.loginfo("Odom Callback: About to call check_position_change (from odom update)..." ) # 新增日志
                    self.check_position_change()
                    # rospy.loginfo("Odom Callback: Returned from check_position_change (from odom update).") # 新增日志
                else:
                    rospy.loginfo("Odom Callback: Skipped position update, using pose estimate.")
                
                # 无论如何都要更新速度信息
                # linear_before = self.linear_vel # 这行可以暂时移除
                # angular_before = self.angular_vel # 这行可以暂时移除
                self.linear_vel = msg.twist.twist.linear.x
                self.angular_vel = msg.twist.twist.angular.z
                # rospy.loginfo(f"Odom Callback: Updated velocity linear={self.linear_vel:.2f}, angular={self.angular_vel:.2f}") # 简化日志
                # rospy.loginfo("Odom Callback: About to call check_velocity_change...") # 新增日志
                self.check_velocity_change()
                # rospy.loginfo("Odom Callback: Returned from check_velocity_change.") # 新增日志
            # rospy.loginfo("Odom Callback: Released data_lock.") # 新增日志
        except Exception as e:
            rospy.logerr(f"处理里程计数据出错: {e}")
    
    def check_position_change(self):
        """检查位置是否发生显著变化"""
        rospy.loginfo("--- check_position_change called ---") # 改为 INFO
        dx = abs(self.position_x - self.last_sent_x)
        dy = abs(self.position_y - self.last_sent_y)
        threshold_exceeded = dx > self.position_threshold or dy > self.position_threshold
        # rospy.loginfo(f"Position check: dx={dx:.4f}, dy={dy:.4f}, thresh={self.position_threshold}, exceeded={threshold_exceeded}") # 改为 INFO
        
        if threshold_exceeded:
            # rospy.loginfo("Position change threshold EXCEEDED. Attempting to add to queue.") # 改为 INFO
            try:
                self.add_data_to_queue()
            except queue.Full:
                rospy.logwarn("Data queue is full, discarding position update.")
        else:
             rospy.loginfo("Position change threshold NOT exceeded.") # 改为 INFO
    
    def check_velocity_change(self):
        """检查速度是否发生显著变化"""
        # rospy.loginfo("--- check_velocity_change called ---") # 改为 INFO
        dlin = abs(self.linear_vel - self.last_sent_linear)
        dang = abs(self.angular_vel - self.last_sent_angular)
        threshold_exceeded = dlin > self.velocity_threshold or dang > self.velocity_threshold
        # rospy.loginfo(f"Velocity check: dlin={dlin:.4f}, dang={dang:.4f}, thresh={self.velocity_threshold}, exceeded={threshold_exceeded}") # 改为 INFO
        
        if threshold_exceeded:
            # rospy.loginfo("Velocity change threshold EXCEEDED. Attempting to add to queue.") # 改为 INFO
            try:
                self.add_data_to_queue()
            except queue.Full:
                rospy.logwarn("Data queue is full, discarding velocity update.")
        else:
            rospy.loginfo("Velocity change threshold NOT exceeded.") # 改为 INFO
    
    def add_data_to_queue(self):
        """将当前状态添加到数据队列"""
        rospy.loginfo("--- add_data_to_queue called --- ")
        rospy.logdebug("Attempting to add data to queue.")
        data_added = False
        with self.data_lock:
            # 仅当至少有一项数据发生显著变化时才添加到队列
            pos_changed = abs(self.position_x - self.last_sent_x) > self.position_threshold or \
                          abs(self.position_y - self.last_sent_y) > self.position_threshold
            vel_changed = abs(self.linear_vel - self.last_sent_linear) > self.velocity_threshold or \
                          abs(self.angular_vel - self.last_sent_angular) > self.velocity_threshold
            bat_changed = abs(self.battery_level - self.last_sent_battery) > self.battery_threshold

            if pos_changed or vel_changed or bat_changed:
                position_data = {
                    'x': self.position_x,
                    'y': self.position_y,
                    'linear_velocity': self.linear_vel,
                    'angular_velocity': self.angular_vel,
                    'battery_level': self.battery_level, # 确保包含 battery_level
                    'status': self.get_robot_status(),
                    'position_source': 'pose_estimate' if self.using_pose_estimate else 'odom'
                }
                try:
                    # 非阻塞方式添加到队列
                    self.data_queue.put(position_data, block=False)
                    rospy.loginfo(f"Successfully added data to queue: {position_data}")
                    # 更新上次发送的值
                    self.last_sent_x = self.position_x
                    self.last_sent_y = self.position_y
                    self.last_sent_linear = self.linear_vel
                    self.last_sent_angular = self.angular_vel
                    self.last_sent_battery = self.battery_level
                    data_added = True
                except queue.Full:
                    rospy.logwarn("Data queue is full, discarding data.")
            else:
                rospy.logdebug("No significant change detected, skipping add_data_to_queue.")

        return data_added # 返回是否添加了数据
    
    def send_position_data(self):
        """发送位置数据到Web服务器"""
        min_interval = 0.2  # 最小发送间隔200ms
        max_interval = 1.0  # 最大发送间隔1秒
        last_send_time = 0
        
        while not rospy.is_shutdown():
            current_time = time.time()
            queue_empty = self.data_queue.empty()
            time_since_last_send = current_time - last_send_time
            should_send_due_to_interval = (not queue_empty and time_since_last_send > min_interval) or \
                                          (time_since_last_send > max_interval)
            
            rospy.logdebug_throttle(5, f"send_position_data check: queue_empty={queue_empty}, time_since_last={time_since_last_send:.2f}s, should_send={should_send_due_to_interval}") # 添加日志

            try:
                # 如果队列有数据且达到最小发送间隔，或者超过最大间隔
                if should_send_due_to_interval:
                    
                    # 尝试从队列获取最新数据
                    try:
                        # 非阻塞获取，如果队列为空则引发异常
                        # 如果是因为超时而发送，即使队列是空的，也要获取最新状态
                        if queue_empty and time_since_last_send > max_interval:
                             rospy.logdebug("Max interval reached, adding current state to queue before sending.")
                             self.add_data_to_queue() # 添加当前状态到队列，下面会取出
                             
                        position_data = self.data_queue.get(block=False) # 现在应该有数据了
                        rospy.logdebug(f"Got data from queue: {position_data}") # 添加日志
                        
                        # 如果连接已建立，发送数据
                        if self.ws_connected and self.ws:
                            # rospy.logdebug("WebSocket connected, preparing to send.") # 添加日志
                            # 修改为调用自定义动作 'update_status'
                            action_cable_message = {
                                "command": "message", # 保持 command 为 message
                                "identifier": json.dumps({"channel": STATUS_CHANNEL}),
                                "data": json.dumps({
                                    "action": "update_status", # 指定要调用的动作
                                    "payload": position_data # 将实际数据放在 payload 中
                                })
                            }

                            try:
                                self.ws.send(json.dumps(action_cable_message))
                                last_send_time = current_time
                                # rospy.loginfo(f"发送 update_status 动作: {position_data}") # 修改日志
                            except websocket.WebSocketConnectionClosedException:
                                rospy.logwarn("WebSocket连接已关闭，无法发送数据")
                                self.ws_connected = False
                            except Exception as e:
                                rospy.logerr(f"发送位置数据出错: {e}")
                        else:
                             rospy.logdebug("WebSocket not connected, skipping send.") # 添加日志
                        
                        # 标记任务完成
                        self.data_queue.task_done()
                        
                    except queue.Empty:
                         # 理论上，如果should_send_due_to_interval为True且不是因为超时，这里不应该发生
                         # 如果是因为超时触发，上面已经添加过数据了
                         rospy.logdebug("Queue was empty unexpectedly in send block.") # 添加日志
                         pass
                    
            except Exception as e:
                rospy.logerr(f"数据发送线程错误: {e}")
            
            # 短暂睡眠，避免CPU占用过高
            time.sleep(0.1)
    
    def send_heartbeat(self):
        """发送心跳保持WebSocket连接"""
        heartbeat_interval = 30  # 心跳间隔30秒
        
        while not rospy.is_shutdown():
            if self.ws_connected and self.ws and self.ws.sock and self.ws.sock.connected:
                try:
                    self.ws.send(json.dumps({"type": "ping"}))
                    if self.debug_mode:
                        rospy.loginfo("已发送心跳")
                except Exception as e:
                    rospy.logwarn(f"发送心跳出错: {e}")
                    self.ws_connected = False
            
            # 心跳间隔
            time.sleep(heartbeat_interval)
    
    def get_robot_status(self):
        """根据当前状态返回机器人状态描述"""
        with self.data_lock:
            if self.battery_level < 10:
                return "low_battery"
            elif abs(self.linear_vel) > 0.1 or abs(self.angular_vel) > 0.1:
                return "moving"
            else:
                return "idle"
    
    def execute_shelf_scan_task(self):
        """执行书架扫描任务"""
        rospy.loginfo("开始执行书架扫描任务")
        
        # 这里应实现实际的导航和扫描逻辑
        # 例如，可以发布一系列预定义的导航目标
        
        # 模拟导航到书架位置
        target_point = (2.0, 1.5)  # 假设的书架坐标
        
        rospy.loginfo(f"导航到书架位置: {target_point}")
        # 实际应用中应该使用move_base等导航功能
        
    def execute_return_base_task(self):
        """执行返回充电站任务"""
        rospy.loginfo("开始执行返回充电站任务")
        
        # 模拟返回充电站
        charging_point = (0.0, 0.0)  # 假设的充电站坐标
        
        rospy.loginfo(f"导航到充电站: {charging_point}")
        # 实际应用中应该使用move_base等导航功能

    # 新增：检查电池电量变化
    def check_battery_change(self):
        """检查电池电量是否发生显著变化"""
        rospy.loginfo("--- check_battery_change called ---")
        dbatt = abs(self.battery_level - self.last_sent_battery)
        threshold_exceeded = dbatt > self.battery_threshold
        rospy.loginfo(f"Battery check: current={self.battery_level:.1f}, last_sent={self.last_sent_battery:.1f}, diff={dbatt:.1f}, thresh={self.battery_threshold}, exceeded={threshold_exceeded}")

        if threshold_exceeded:
            rospy.loginfo("Battery change threshold EXCEEDED. Attempting to add to queue.")
            try:
                self.add_data_to_queue()
            except queue.Full:
                 rospy.logwarn("Data queue is full, discarding battery update.")
        else:
            rospy.loginfo("Battery change threshold NOT exceeded.")

    # 新增：模拟电池消耗的线程函数
    def simulate_battery_drain(self):
        """模拟电池电量随时间缓慢下降"""
        drain_interval = 5  # 每 5 秒更新一次电量
        min_drain = 0.1     # 每次最少消耗 0.1%
        max_drain = 0.5     # 每次最多消耗 0.5%

        while not rospy.is_shutdown():
            time.sleep(drain_interval)
            with self.data_lock:
                if self.battery_level > 0:
                    drain_amount = random.uniform(min_drain, max_drain)
                    self.battery_level -= drain_amount
                    self.battery_level = max(0, self.battery_level) # 确保不低于 0
                    rospy.loginfo(f"[Battery Sim] Drained {drain_amount:.2f}%, current level: {self.battery_level:.1f}%")
                else:
                     rospy.loginfo("[Battery Sim] Battery level is 0.")

                # 检查电池电量变化是否需要发送更新
                self.check_battery_change()

if __name__ == '__main__':
    try:
        # 先初始化ROS节点
        rospy.init_node('web_robot_bridge')
        
        # 设置日志级别
        log_level = rospy.get_param('~log_level', rospy.INFO)
        rospy.loginfo(f"设置日志级别: {log_level}")
        
        # 创建节点实例
        node = WebRobotBridgeNode()
        
        # 进入ROS主循环
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Web Robot Bridge 节点已关闭")
    except Exception as e:
        rospy.logerr(f"Web Robot Bridge 节点错误: {e}")
        raise