# 图书馆机器人摄像头系统设计文档

## 系统概述

本文档描述了为图书馆机器人控制系统添加的摄像头画面传输功能。该系统基于现有的ROS-Rails架构，实现了实时摄像头数据的采集、处理、传输和显示。

## 系统架构

### 整体数据流
```
摄像头设备 → Camera Publisher Node → Web Robot Bridge Node → Rails ActionCable → 前端显示
     ↑                                                                    ↓
     └─────────────── 摄像头控制指令 ←─────────────────────────────────────┘
```

### 组件说明

#### 1. ROS端组件

**Camera Publisher Node** (`library_robot_interfaces/scripts/camera_publisher_node.py`)
- **功能**: 采集摄像头数据，处理图像，发布到ROS topic
- **特性**: 
  - 自动检测并订阅压缩或原始图像topic
  - 图像尺寸调整和JPEG压缩
  - 可配置的帧率和图像质量
  - 时间戳水印添加
- **发布topic**: `/robot_core/camera_stream` (CameraFrameData消息)
- **订阅topic**: `/camera_control/command` (摄像头控制指令)

**Web Robot Bridge Node** (修改版 `library_robot_interfaces/scripts/web_robot_bridge_node.py`)
- **功能**: ROS与Rails之间的桥接，负责摄像头数据转发
- **新增特性**:
  - 订阅摄像头数据流
  - 帧率限制和流量控制
  - 摄像头控制指令转发
- **订阅topic**: `/robot_core/camera_stream`
- **发布topic**: `/camera_control/command`

#### 2. Rails端组件

**CameraStreamChannel** (`app/channels/camera_stream_channel.rb`)
- **功能**: 处理摄像头数据流的接收和分发
- **特性**:
  - 接收来自ROS的摄像头帧数据
  - 数据验证和大小限制
  - 广播给所有订阅用户
  - 摄像头参数控制

**RobotControlChannel** (修改版 `app/channels/robot_control_channel.rb`)
- **新增功能**: `toggle_camera_stream` action
- **特性**: 摄像头开关控制和参数调整

#### 3. 前端组件

**CameraStreamChannel.js** (`app/javascript/channels/camera_stream_channel.js`)
- **功能**: 前端摄像头流接口
- **特性**:
  - 接收实时摄像头帧数据
  - 摄像头控制事件分发
  - 连接状态管理

**Robot Control Controller** (修改版 `app/javascript/controllers/robot_control_controller.js`)
- **新增功能**: 摄像头控制和显示逻辑
- **特性**:
  - 摄像头流切换
  - 实时画面显示
  - 帧率和质量控制
  - 统计信息计算

**控制页面视图** (修改版 `app/views/robots/control.html.erb`)
- **新增区域**: 摄像头监控面板
- **特性**:
  - 实时画面显示
  - 控制按钮和参数滑块
  - 状态指示器

## 技术实现细节

### 数据格式

**ROS消息: CameraFrameData**
```msg
std_msgs/Header header
int32 frame_id
string format          # "jpeg"
int32 width
int32 height  
string data_base64     # Base64编码的图像数据
int32 data_size        # 原始数据大小（字节）
```

**ActionCable传输格式**
```json
{
  "type": "camera_frame",
  "frame_id": 12345,
  "timestamp": 1640995200.123,
  "format": "jpeg",
  "width": 640,
  "height": 480,
  "data_url": "data:image/jpeg;base64,/9j/4AAQ...",
  "data_size": 15432
}
```

### 性能优化

1. **图像压缩**: 
   - JPEG格式，可调质量（10-100%）
   - 目标分辨率640x480
   - Base64编码传输

2. **帧率控制**:
   - ROS端: 可配置目标帧率（1-15 FPS）
   - Bridge端: 200ms限流间隔
   - 前端: 平滑FPS计算

3. **流量管理**:
   - 1MB单帧大小限制
   - WebSocket连接状态检查
   - 自动重连机制

### 安全性

1. **访问控制**:
   - 用户身份验证
   - 管理员权限检查（参数调整）
   - API密钥验证（ROS客户端）

2. **数据验证**:
   - 必填字段检查
   - 数据大小限制
   - 格式验证

## 使用方法

### 启动摄像头节点
```bash
# 在ROS环境中启动摄像头发布节点
rosrun library_robot_interfaces camera_publisher_node.py

# 或通过launch文件启动
roslaunch library_robot_interfaces camera_system.launch
```

### 前端操作
1. 访问机器人控制台页面
2. 在摄像头监控面板点击"开启摄像头"
3. 调整帧率和图像质量参数
4. 查看实时画面

### 控制指令
- **开启摄像头**: `RobotControlChannel.toggleCameraStream(true)`
- **关闭摄像头**: `RobotControlChannel.toggleCameraStream(false)`
- **调整参数**: `CameraStreamChannel.setCameraParams({fps: 10, quality: 80})`

## 配置参数

### ROS参数 (camera_publisher_node)
```yaml
camera_fps: 5.0           # 目标帧率
frame_width: 640          # 图像宽度
frame_height: 480         # 图像高度
jpeg_quality: 70          # JPEG压缩质量
```

### Rails配置
```ruby
# config/cable.yml - ActionCable配置
development:
  adapter: redis
  url: redis://localhost:6379/1
```

## 故障排除

### 常见问题

1. **摄像头无法开启**
   - 检查摄像头设备连接
   - 确认ROS摄像头topic可用
   - 验证权限设置

2. **画面延迟或卡顿**
   - 降低帧率设置
   - 减少图像质量
   - 检查网络带宽

3. **连接断开**
   - 检查WebSocket连接
   - 确认Redis服务状态
   - 验证API密钥配置

### 调试命令

```bash
# 检查ROS摄像头topic
rostopic list | grep camera
rostopic echo /camera/image_raw/compressed

# 检查摄像头节点状态
rosnode info /camera_publisher_node

# 查看Rails日志
tail -f log/development.log | grep Camera
```

## 扩展功能

### 计划中的功能
1. **多摄像头支持**: 支持多个摄像头设备
2. **录制功能**: 视频录制和回放
3. **图像分析**: 集成目标检测和识别
4. **云存储**: 图像数据云端备份

### 性能监控
- 实时FPS显示
- 网络带宽监控
- 延迟统计
- 错误率追踪

## 结论

摄像头系统成功集成到现有的图书馆机器人控制系统中，提供了：
- 实时摄像头画面传输
- 灵活的参数控制
- 用户友好的操作界面
- 可扩展的架构设计

该系统遵循了现有的ROS-Rails架构模式，确保了代码的一致性和可维护性。 