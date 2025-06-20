#!/usr/bin/env python3
import rospy
import cv2
import base64
import threading
import time
import json
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from library_robot_interfaces.msg import CameraFrameData

# ROS Topics - 适配启智机器人Kinect2摄像头
CAMERA_RAW_TOPIC = "/kinect2/hd/image_color_rect"           # 启智机器人Kinect2高清原始彩色图像
CAMERA_COMPRESSED_TOPIC = "/kinect2/hd/image_color_rect/compressed"  # 启智机器人Kinect2高清压缩彩色图像
CAMERA_STREAM_TOPIC = "/robot_core/camera_stream"  # 发布处理后的摄像头数据

# 备选摄像头话题（如果高清不可用）
CAMERA_QHD_RAW_TOPIC = "/kinect2/qhd/image_color_rect"      # 中等分辨率
CAMERA_QHD_COMPRESSED_TOPIC = "/kinect2/qhd/image_color_rect/compressed"

class CameraPublisherNode:
    def __init__(self):
        # 注意：当通过launch文件启动时，不要调用rospy.init_node()
        # rospy.init_node('camera_publisher_node', anonymous=True)
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing Camera Publisher for 启智机器人 Kinect2...")

        # 内部状态
        self.camera_streaming_enabled = False
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.cv_bridge = CvBridge()
        
        # 配置参数
        self.target_fps = rospy.get_param("~camera_fps", 10.0)  # 降低帧率以减少网络负载
        self.frame_width = rospy.get_param("~frame_width", 640)
        self.frame_height = rospy.get_param("~frame_height", 480)
        self.jpeg_quality = rospy.get_param("~jpeg_quality", 80)  # JPEG压缩质量
        
        # 定时器和线程
        self.processing_thread = None
        self.last_publish_time = rospy.Time.now()
        self.publish_rate = 1.0 / self.target_fps

        # ROS Publishers and Subscribers
        self.camera_stream_pub = rospy.Publisher(CAMERA_STREAM_TOPIC, CameraFrameData, queue_size=5)
        
        # 优先订阅压缩图像（如果可用），否则使用原始图像
        # 首先尝试高清压缩图像
        try:
            rospy.wait_for_message(CAMERA_COMPRESSED_TOPIC, CompressedImage, timeout=3.0)
            rospy.Subscriber(CAMERA_COMPRESSED_TOPIC, CompressedImage, self._compressed_image_callback, queue_size=2)
            rospy.loginfo(f"[{self.node_name}] Subscribed to Kinect2 HD compressed camera topic: {CAMERA_COMPRESSED_TOPIC}")
        except rospy.ROSException:
            rospy.logwarn(f"[{self.node_name}] HD compressed camera topic not available, trying HD raw topic...")
            # 尝试高清原始图像
            try:
                rospy.wait_for_message(CAMERA_RAW_TOPIC, Image, timeout=3.0)
                rospy.Subscriber(CAMERA_RAW_TOPIC, Image, self._raw_image_callback, queue_size=2)
                rospy.loginfo(f"[{self.node_name}] Subscribed to Kinect2 HD raw camera topic: {CAMERA_RAW_TOPIC}")
            except rospy.ROSException:
                rospy.logwarn(f"[{self.node_name}] HD raw camera topic not available, trying QHD topics...")
                # 尝试中等分辨率压缩图像
                try:
                    rospy.wait_for_message(CAMERA_QHD_COMPRESSED_TOPIC, CompressedImage, timeout=3.0)
                    rospy.Subscriber(CAMERA_QHD_COMPRESSED_TOPIC, CompressedImage, self._compressed_image_callback, queue_size=2)
                    rospy.loginfo(f"[{self.node_name}] Subscribed to Kinect2 QHD compressed camera topic: {CAMERA_QHD_COMPRESSED_TOPIC}")
                except rospy.ROSException:
                    # 最后尝试中等分辨率原始图像
                    try:
                        rospy.wait_for_message(CAMERA_QHD_RAW_TOPIC, Image, timeout=3.0)
                        rospy.Subscriber(CAMERA_QHD_RAW_TOPIC, Image, self._raw_image_callback, queue_size=2)
                        rospy.loginfo(f"[{self.node_name}] Subscribed to Kinect2 QHD raw camera topic: {CAMERA_QHD_RAW_TOPIC}")
                    except rospy.ROSException:
                        rospy.logwarn(f"[{self.node_name}] No Kinect2 camera topics available. Camera functionality will be limited.")

        # 控制命令订阅（接收开启/关闭流的指令）
        rospy.Subscriber("/camera_control/command", String, self._camera_control_callback, queue_size=5)

        # 启动处理线程
        self._start_processing_thread()
        
        rospy.loginfo(f"[{self.node_name}] Kinect2 Camera Publisher initialized.")

    def _start_processing_thread(self):
        if self.processing_thread and self.processing_thread.is_alive():
            return
        self.processing_thread = threading.Thread(target=self._camera_processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def _camera_processing_loop(self):
        """主处理循环，负责定期发布摄像头数据"""
        rate = rospy.Rate(self.target_fps)
        
        while not rospy.is_shutdown():
            if self.camera_streaming_enabled:
                current_time = rospy.Time.now()
                time_since_last = (current_time - self.last_publish_time).to_sec()
                
                if time_since_last >= self.publish_rate:
                    self._publish_current_frame()
                    self.last_publish_time = current_time
            
            rate.sleep()

    def _compressed_image_callback(self, msg):
        """处理压缩图像数据"""
        if not self.camera_streaming_enabled:
            return
            
        try:
            # 将压缩图像转换为OpenCV格式
            import numpy as np
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                with self.frame_lock:
                    self.current_frame = self._process_frame(cv_image)
                    
        except Exception as e:
            rospy.logwarn(f"[{self.node_name}] Error processing compressed image: {e}")

    def _raw_image_callback(self, msg):
        """处理原始图像数据"""
        if not self.camera_streaming_enabled:
            return
            
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.frame_lock:
                self.current_frame = self._process_frame(cv_image)
                
        except CvBridgeError as e:
            rospy.logwarn(f"[{self.node_name}] Error converting image: {e}")

    def _process_frame(self, cv_image):
        """处理单帧图像：调整大小、压缩等"""
        try:
            # 调整图像尺寸
            if cv_image.shape[1] != self.frame_width or cv_image.shape[0] != self.frame_height:
                cv_image = cv2.resize(cv_image, (self.frame_width, self.frame_height), interpolation=cv2.INTER_AREA)
            
            # 可选：添加时间戳水印
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(cv_image, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 添加摄像头信息水印
            camera_info = f"Kinect2: {self.frame_width}x{self.frame_height} @ {self.target_fps:.1f}fps"
            cv2.putText(cv_image, camera_info, (10, cv_image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            return cv_image
            
        except Exception as e:
            rospy.logwarn(f"[{self.node_name}] Error processing frame: {e}")
            return cv_image

    def _publish_current_frame(self):
        """发布当前帧到ROS topic"""
        with self.frame_lock:
            if self.current_frame is None:
                return
                
            try:
                # 将OpenCV图像编码为JPEG
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                result, encoded_img = cv2.imencode('.jpg', self.current_frame, encode_param)
                
                if not result:
                    rospy.logwarn(f"[{self.node_name}] Failed to encode frame as JPEG")
                    return
                
                # 转换为base64字符串
                img_base64 = base64.b64encode(encoded_img.tobytes()).decode('utf-8')
                
                # 创建ROS消息
                camera_msg = CameraFrameData()
                camera_msg.header.stamp = rospy.Time.now()
                camera_msg.header.frame_id = "kinect2_rgb_optical_frame"  # 使用Kinect2的frame_id
                camera_msg.frame_id = int(time.time() * 1000) % 1000000  # 简单的帧ID
                camera_msg.format = "jpeg"
                camera_msg.width = self.frame_width
                camera_msg.height = self.frame_height
                camera_msg.data_base64 = img_base64
                camera_msg.data_size = len(encoded_img.tobytes())
                
                # 发布消息
                self.camera_stream_pub.publish(camera_msg)
                rospy.logdebug(f"[{self.node_name}] Published Kinect2 camera frame: {camera_msg.data_size} bytes")
                
            except Exception as e:
                rospy.logerr(f"[{self.node_name}] Error publishing camera frame: {e}")

    def _camera_control_callback(self, msg):
        """处理摄像头控制指令"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get("command_type")
            
            if command_type == "TOGGLE_CAMERA_STREAM":
                enable = command_data.get("enable", False)
                self._toggle_camera_stream(enable)
            elif command_type == "SET_CAMERA_PARAMS":
                self._update_camera_parameters(command_data.get("params", {}))
            else:
                rospy.logwarn(f"[{self.node_name}] Unknown camera command: {command_type}")
                
        except json.JSONDecodeError:
            rospy.logwarn(f"[{self.node_name}] Invalid JSON in camera control message: {msg.data}")
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing camera control command: {e}")

    def _toggle_camera_stream(self, enable):
        """开启或关闭摄像头流"""
        if enable and not self.camera_streaming_enabled:
            self.camera_streaming_enabled = True
            rospy.loginfo(f"[{self.node_name}] Kinect2 camera streaming ENABLED")
        elif not enable and self.camera_streaming_enabled:
            self.camera_streaming_enabled = False
            with self.frame_lock:
                self.current_frame = None
            rospy.loginfo(f"[{self.node_name}] Kinect2 camera streaming DISABLED")

    def _update_camera_parameters(self, params):
        """更新摄像头参数"""
        if "fps" in params:
            new_fps = max(1.0, min(15.0, float(params["fps"])))  # 限制在合理范围内
            if new_fps != self.target_fps:
                self.target_fps = new_fps
                self.publish_rate = 1.0 / self.target_fps
                rospy.loginfo(f"[{self.node_name}] Updated Kinect2 camera FPS to {self.target_fps}")
        
        if "quality" in params:
            new_quality = max(10, min(100, int(params["quality"])))
            if new_quality != self.jpeg_quality:
                self.jpeg_quality = new_quality
                rospy.loginfo(f"[{self.node_name}] Updated JPEG quality to {self.jpeg_quality}")

if __name__ == '__main__':
    camera_node_instance = None
    try:
        # 当作为独立脚本运行时初始化节点
        rospy.init_node('camera_publisher_node', anonymous=False)
        import numpy as np  # cv2需要numpy
        camera_node_instance = CameraPublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"[{camera_node_instance.node_name if camera_node_instance else 'camera_publisher'}] Shutting down.")
    finally:
        node_name_for_log = camera_node_instance.node_name if camera_node_instance and hasattr(camera_node_instance, 'node_name') else 'camera_publisher'
        rospy.loginfo(f"[{node_name_for_log}] Cleanup complete. Exiting.")

# --- 自定义ROS消息定义 (应放在项目的 msg 目录下) ---
# library_robot_interfaces/msg/CameraFrameData.msg
"""
std_msgs/Header header
int32 frame_id
string format          # e.g., "jpeg", "png"
int32 width
int32 height
string data_base64     # Base64编码的图像数据
int32 data_size        # 原始数据大小（字节）
""" 