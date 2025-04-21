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

# 使用localhost因为是本地测试
WEB_SERVER_URL = "http://localhost:3000"
# 添加API密钥到WebSocket URL
WEBSOCKET_URL = "ws://localhost:3000/cable"
API_KEY = "7ad0bbbdf00c5cbe87799355200f212ed329030028fd3ccd51524e461adf2c31"

STATUS_CHANNEL = "RobotStatusChannel"
COMMAND_CHANNEL = "RobotCommandChannel"

ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"
POSE_TOPIC = "/amcl_pose"  # AMCL或其他pose estimation发布的话题

class WebRobotBridgeNode:
    def __init__(self):
        """初始化ROS节点和必要的变量"""
        # 设置日志级别
        self.debug_mode = rospy.get_param('~debug_mode', False)

        # 创建线程锁
        self.data_lock = threading.Lock()
        
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
        self.position_threshold = 0.01  # 1厘米
        self.velocity_threshold = 0.01  # 0.01 m/s或rad/s
        
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
        
        # 订阅pose estimation话题
        rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback)
        
        # WebSocket连接对象
        self.ws = None
        self.ws_connected = False
        
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
        
        rospy.loginfo("Web Robot Bridge 节点启动成功！")

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
                WEBSOCKET_URL + "?api_key=" + API_KEY,
                header=headers,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close,
                on_open=self.on_open
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
        """处理pose estimation话题的回调，如来自AMCL的位姿估计"""
        try:
            with self.data_lock:
                if not self.using_pose_estimate:  # 避免与TF线程冲突
                    self.position_x = msg.pose.pose.position.x
                    self.position_y = msg.pose.pose.position.y
                    self.using_pose_estimate = True
                    self.check_position_change()
                    rospy.loginfo(f"从pose estimation更新位置: x={self.position_x:.2f}, y={self.position_y:.2f}")
        except Exception as e:
            rospy.logerr(f"处理pose estimation数据出错: {e}")
    
    def on_open(self, ws):
        """WebSocket连接成功时的回调"""
        self.ws_connected = True
        rospy.loginfo("WebSocket连接成功")
        
        # 发送订阅消息
        subscription = {
            "command": "subscribe",
            "identifier": json.dumps({"channel": COMMAND_CHANNEL}),
        }
        try:
            ws.send(json.dumps(subscription))
            rospy.loginfo("已订阅RobotCommandChannel: %s", json.dumps(subscription))
        except Exception as e:
            rospy.logerr(f"订阅频道出错: {e}")
    
    def on_message(self, ws, message):
        """处理从WebSocket接收到的消息"""
        try:
            if self.debug_mode:
                rospy.loginfo(f"收到WebSocket消息: {message}")
            data = json.loads(message)
            
            # 确认消息处理
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
            if 'message' in data and isinstance(data['message'], dict):
                cmd_data = data['message']
                cmd_type = cmd_data.get('command')
                
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
                    self.cmd_vel_pub.publish(twist)
                    
                    rospy.loginfo(f"发送移动命令: direction={direction}, speed={speed}, linear_x={linear_x}, angular_z={angular_z}")
                
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
        try:
            with self.data_lock:
                # 只有在没有更高优先级的位置源时才使用里程计位置数据
                if not self.using_pose_estimate:
                    self.position_x = msg.pose.pose.position.x
                    self.position_y = msg.pose.pose.position.y
                    self.check_position_change()
                    rospy.logdebug(f"从odom更新位置: x={self.position_x:.2f}, y={self.position_y:.2f}")
                
                # 无论如何都要更新速度信息
                self.linear_vel = msg.twist.twist.linear.x
                self.angular_vel = msg.twist.twist.angular.z
                self.check_velocity_change()
        except Exception as e:
            rospy.logerr(f"处理里程计数据出错: {e}")
    
    def check_position_change(self):
        """检查位置是否发生显著变化，如果是则向队列添加数据"""
        dx = abs(self.position_x - self.last_sent_x)
        dy = abs(self.position_y - self.last_sent_y)
        
        if dx > self.position_threshold or dy > self.position_threshold:
            try:
                # 仅在位置显著变化时添加到队列
                self.add_data_to_queue()
                self.last_sent_x = self.position_x
                self.last_sent_y = self.position_y
            except queue.Full:
                rospy.logwarn("数据队列已满，丢弃位置更新")
    
    def check_velocity_change(self):
        """检查速度是否发生显著变化，如果是则向队列添加数据"""
        dlin = abs(self.linear_vel - self.last_sent_linear)
        dang = abs(self.angular_vel - self.last_sent_angular)
        
        if dlin > self.velocity_threshold or dang > self.velocity_threshold:
            try:
                # 仅在速度显著变化时添加到队列
                self.add_data_to_queue()
                self.last_sent_linear = self.linear_vel
                self.last_sent_angular = self.angular_vel
            except queue.Full:
                rospy.logwarn("数据队列已满，丢弃速度更新")
    
    def add_data_to_queue(self):
        """将当前状态添加到数据队列"""
        with self.data_lock:
            position_data = {
                'x': self.position_x,
                'y': self.position_y,
                'linear_velocity': self.linear_vel,
                'angular_velocity': self.angular_vel,
                'battery': self.battery_level,
                'status': self.get_robot_status(),
                # 移除API密钥，不再在每条消息中包含
                'position_source': 'pose_estimate' if self.using_pose_estimate else 'odom'
            }
        
        # 非阻塞方式添加到队列
        self.data_queue.put(position_data, block=False)
    
    def send_position_data(self):
        """发送位置数据到Web服务器"""
        min_interval = 0.2  # 最小发送间隔200ms
        max_interval = 1.0  # 最大发送间隔1秒
        last_send_time = 0
        
        while not rospy.is_shutdown():
            current_time = time.time()
            
            try:
                # 如果队列有数据且达到最小发送间隔，或者超过最大间隔
                if (not self.data_queue.empty() and 
                    current_time - last_send_time > min_interval) or \
                   current_time - last_send_time > max_interval:
                    
                    # 尝试从队列获取最新数据
                    try:
                        # 非阻塞获取，如果队列为空则引发异常
                        position_data = self.data_queue.get(block=False)
                        
                        # 如果连接已建立，发送数据
                        if self.ws_connected and self.ws:
                            action_cable_message = {
                                "command": "message",
                                "identifier": json.dumps({"channel": STATUS_CHANNEL}),
                                "data": json.dumps(position_data)
                            }

                            try:
                                self.ws.send(json.dumps(action_cable_message))
                                last_send_time = current_time
                                if self.debug_mode:
                                    rospy.loginfo(f"发送位置数据: {position_data}")
                            except websocket.WebSocketConnectionClosedException:
                                rospy.logwarn("WebSocket连接已关闭，无法发送数据")
                                self.ws_connected = False
                            except Exception as e:
                                rospy.logerr(f"发送位置数据出错: {e}")
                        
                        # 标记任务完成
                        self.data_queue.task_done()
                        
                    except queue.Empty:
                        # 如果超过最大间隔且队列为空，发送一次当前状态
                        if current_time - last_send_time > max_interval:
                            self.add_data_to_queue()  # 添加当前状态到队列
                            continue  # 继续循环，下次迭代会处理这个新添加的数据
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