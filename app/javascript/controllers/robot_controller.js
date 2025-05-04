import { Controller } from "@hotwired/stimulus"
import consumer from "channels/consumer"

export default class extends Controller {
  static targets = [
    "statusIndicator", "statusText", "batteryBar", "batteryText", 
    "positionX", "positionY", "linearVelocity", "angularVelocity", 
    "connectionStatus", "speedSlider", "speedValue"
  ]
  
  connect() {
    console.log("Robot控制器已连接")
    this.speedValue = 0.5 // 默认速度
    this.speedValueTarget.textContent = this.speedValue
    
    // 更新速度值显示
    this.speedSliderTarget.addEventListener("input", (event) => {
      this.speedValue = parseFloat(event.target.value)
      this.speedValueTarget.textContent = this.speedValue
    })
    
    this.statusConnected = false // 添加状态标志
    this.commandConnected = false // 添加状态标志
    this.updateConnectionStatusText()
    
    // 连接到RobotStatusChannel
    this.statusSubscription = consumer.subscriptions.create("RobotStatusChannel", {
      connected: this._statusConnected.bind(this),
      disconnected: this._statusDisconnected.bind(this),
      received: this._statusReceived.bind(this)
    })
    
    // 连接到RobotCommandChannel
    this.commandSubscription = consumer.subscriptions.create("RobotCommandChannel", {
      connected: this._commandConnected.bind(this),
      disconnected: this._commandDisconnected.bind(this),
      received: this._commandReceived.bind(this)
    })
  }
  
  disconnect() {
    // 断开WebSocket连接
    if (this.statusSubscription) {
      this.statusSubscription.unsubscribe()
    }
    if (this.commandSubscription) {
      this.commandSubscription.unsubscribe()
    }
    this.statusConnected = false
    this.commandConnected = false
    this.updateConnectionStatusText()
  }
  
  // 状态频道连接处理
  _statusConnected() {
    console.log("已连接到RobotStatusChannel")
    this.statusConnected = true
    this.updateConnectionStatusText()
  }
  
  _statusDisconnected() {
    console.log("已断开RobotStatusChannel")
    this.statusConnected = false
    this.updateConnectionStatusText()
    this._updateStatusIndicator("disconnected")
  }
  
  _statusReceived(data) {
    console.log("收到机器人状态:", data) // 确认数据到达
    
    try { // 包裹在 try...catch 中捕获潜在错误
      // 更新状态指示器
      if (data.heartbeat || data.status) { // 检查 heartbeat 或 status 字段
        // 假设 'online' 是通过心跳或其他状态字段判断，这里简单处理
        this._updateStatusIndicator("online") 
        console.log("[UpdateUI] Status indicator updated based on received data.")
      } else {
        // 如果长时间没收到心跳或明确的离线状态，可以在别处处理
      }
      
      // 更新电池电量
      if (data.battery_level !== undefined) {
        const batteryLevel = parseFloat(data.battery_level).toFixed(3)
        console.log(`[UpdateUI] Attempting to update battery: ${batteryLevel}%`)
        this.batteryTextTarget.textContent = `${batteryLevel}%`
        this.batteryBarTarget.style.width = `${batteryLevel}%`
        
        // 根据电量改变颜色
        if (batteryLevel > 50) {
          this.batteryBarTarget.classList.remove("bg-yellow-500", "bg-red-500")
          this.batteryBarTarget.classList.add("bg-green-500")
        } else if (batteryLevel > 20) {
          this.batteryBarTarget.classList.remove("bg-green-500", "bg-red-500")
          this.batteryBarTarget.classList.add("bg-yellow-500")
        } else {
          this.batteryBarTarget.classList.remove("bg-green-500", "bg-yellow-500")
          this.batteryBarTarget.classList.add("bg-red-500")
        }
        console.log("[UpdateUI] Battery updated.")
      } else {
        console.log("[UpdateUI] battery_level missing in data.")
      }
      
      // 更新位置信息
      if (data.x !== undefined) { // ROS 节点发送的是 'x'
        console.log(`[UpdateUI] Attempting to update position X: ${data.x}`)
        this.positionXTarget.textContent = data.x.toFixed(2)
        console.log("[UpdateUI] Position X updated.")
      } else {
         console.log("[UpdateUI] position_x (or x) missing in data.")
      }
      
      if (data.y !== undefined) { // ROS 节点发送的是 'y'
        console.log(`[UpdateUI] Attempting to update position Y: ${data.y}`)
        this.positionYTarget.textContent = data.y.toFixed(2)
        console.log("[UpdateUI] Position Y updated.")
      } else {
         console.log("[UpdateUI] position_y (or y) missing in data.")
      }
      
      // 更新速度信息 - 使用正确的键名
      if (data.linear_velocity !== undefined) { // 使用 ROS 发送的键名
        console.log(`[UpdateUI] Attempting to update linear velocity: ${data.linear_velocity}`)
        this.linearVelocityTarget.textContent = data.linear_velocity.toFixed(2)
        console.log("[UpdateUI] Linear velocity updated.")
      } else {
         console.log("[UpdateUI] linear_velocity missing in data.")
      }
      
      if (data.angular_velocity !== undefined) { // 使用 ROS 发送的键名
        console.log(`[UpdateUI] Attempting to update angular velocity: ${data.angular_velocity}`)
        this.angularVelocityTarget.textContent = data.angular_velocity.toFixed(2)
        console.log("[UpdateUI] Angular velocity updated.")
      } else {
         console.log("[UpdateUI] angular_velocity missing in data.")
      }
    } catch (e) {
        console.error("Error processing received status data:", e)
    }
  }
  
  // 命令频道连接处理
  _commandConnected() {
    console.log("已连接到RobotCommandChannel")
    this.commandConnected = true
    this.updateConnectionStatusText()
  }
  
  _commandDisconnected() {
    console.log("已断开RobotCommandChannel")
    this.commandConnected = false
    this.updateConnectionStatusText()
  }
  
  _commandReceived(data) {
    console.log("收到命令确认:", data)
  }
  
  // 状态指示器更新
  _updateStatusIndicator(status) {
    const indicator = this.statusIndicatorTarget
    const statusText = this.statusTextTarget
    
    // 移除所有状态类
    indicator.classList.remove("bg-green-400", "bg-red-400", "bg-yellow-400", "bg-gray-400")
    
    // 根据状态设置样式
    switch (status) {
      case "online":
        indicator.classList.add("bg-green-400")
        statusText.textContent = "在线"
        break
      case "offline":
        indicator.classList.add("bg-red-400")
        statusText.textContent = "离线"
        break
      case "warning":
        indicator.classList.add("bg-yellow-400")
        statusText.textContent = "警告"
        break
      case "disconnected":
      default:
        indicator.classList.add("bg-gray-400")
        statusText.textContent = "未连接"
        break
    }
  }
  
  // 控制命令方法
  moveForward(event) {
    this._sendMoveCommand("forward")
  }
  
  moveBackward(event) {
    this._sendMoveCommand("backward")
  }
  
  moveLeft(event) {
    this._sendMoveCommand("left")
  }
  
  moveRight(event) {
    this._sendMoveCommand("right")
  }
  
  stop(event) {
    this._sendMoveCommand("stop")
  }
  
  // 发送移动命令
  _sendMoveCommand(direction) {
    if (this.commandSubscription) {
      const commandData = { 
        direction: direction, 
        speed: this.speedValue 
      };
      // 添加日志确认发送的数据
      console.log(`[Command] Sending command via ActionCable perform:`, commandData);
      this.commandSubscription.perform("move", commandData);
    } else {
      console.error("[Command] 命令频道未连接，无法发送命令");
    }
  }
  
  // 新增: 更新连接状态文本的辅助函数
  updateConnectionStatusText() {
    if (this.statusConnected && this.commandConnected) {
      this.connectionStatusTarget.textContent = "双向通信已连接"
      this.connectionStatusTarget.classList.remove("text-red-600", "text-yellow-600")
      this.connectionStatusTarget.classList.add("text-green-600")
    } else if (this.statusConnected) {
      this.connectionStatusTarget.textContent = "状态频道已连接 / 命令频道未连接"
      this.connectionStatusTarget.classList.remove("text-red-600", "text-green-600")
      this.connectionStatusTarget.classList.add("text-yellow-600")
    } else if (this.commandConnected) {
      this.connectionStatusTarget.textContent = "命令频道已连接 / 状态频道未连接"
      this.connectionStatusTarget.classList.remove("text-red-600", "text-green-600")
      this.connectionStatusTarget.classList.add("text-yellow-600")
    } else {
      this.connectionStatusTarget.textContent = "连接已断开"
      this.connectionStatusTarget.classList.remove("text-yellow-600", "text-green-600")
      this.connectionStatusTarget.classList.add("text-red-600")
    }
  }
} 