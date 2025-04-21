# 图书馆机器人控制系统测试指南

## 系统架构简介

本系统使用Rails 7.2的ActionCable功能实现了与ROS机器人的WebSocket通信。架构如下：

1. **Rails Web应用**：提供前端界面，并通过ActionCable建立WebSocket通信
2. **ROS中间层节点**：运行在机器人上，连接到Rails的WebSocket服务，并与ROS系统通信
3. **ROS机器人节点**：机器人的各个功能节点

## 测试方法

### 方法一：使用模拟端点

系统内置了一个模拟机器人状态的端点，可以生成随机的机器人状态数据用于测试：

```bash
# 访问以下URL来生成一次随机状态数据
curl http://localhost:3000/robots/simulate_status
```

或者使用浏览器直接访问 `/robots/simulate_status` 路径。

### 方法二：使用POST请求模拟机器人状态更新

可以通过发送POST请求到 `/robots/update_status` 端点来模拟ROS节点发送的状态数据：

```bash
curl -X POST \
  http://localhost:3000/robots/update_status \
  -H 'Content-Type: application/json' \
  -H 'X-Robot-API-Key: 7ad0bbbdf00c5cbe87799355200f212ed329030028fd3ccd51524e461adf2c31' \
  -d '{
    "heartbeat": true,
    "battery_level": 85,
    "position_x": 2.5,
    "position_y": 1.8,
    "linear_vel": 0.2,
    "angular_vel": 0.0
  }'
```

### 方法三：运行ROS中间层节点

如果你已经有一个ROS环境，可以使用项目中的`web_robot_bridge_node.py`：

1. 确保ROS环境已经设置好
2. 将`web_robot_bridge_node.py`复制到你的ROS包的`scripts`目录
3. 确保脚本有执行权限：`chmod +x web_robot_bridge_node.py`
4. 更新脚本中的WebSocket URL，指向你的Rails应用（本地开发时是`ws://localhost:3000/cable`）
5. 运行节点：`rosrun your_package_name web_robot_bridge_node.py`

## 测试控制功能

在机器人控制页面（`/robots/control`）上：

1. 使用方向按钮发送移动命令
2. 使用速度滑块调整移动速度
3. 使用停止按钮发送停止命令

这些命令将通过WebSocket发送到ROS中间层，并最终控制机器人。

## 故障排除

1. **WebSocket连接问题**：
   - 检查浏览器控制台是否有错误信息
   - 确认Redis服务正常运行
   - 检查Rails服务器日志

2. **命令未被接收**：
   - 确认WebSocket连接已建立
   - 检查ROS节点是否在线

3. **状态未更新**：
   - 检查是否收到了来自ROS的状态数据
   - 尝试使用模拟端点生成测试数据 