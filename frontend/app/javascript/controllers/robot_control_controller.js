import { Controller } from "@hotwired/stimulus"
import consumer from "channels/consumer"
import RobotTaskChannel from "channels/robot_task_channel"
import RobotControlChannel from "channels/robot_control_channel"
import RobotFeedbackInterface from "channels/robot_feedback_channel"
import CameraStreamChannel from "channels/camera_stream_channel"

/**
 * 机器人控制控制器
 * 
 * 建图流程说明：
 * 1. 用户点击"开始建图" -> 调用 startMapping() -> 创建 map_build_auto 任务 (通过 RobotTaskChannel)
 * 2. 机器人开始自动建图，任务状态变为 'in_progress'
 * 3. 用户想要结束建图时点击"完成建图" -> 调用 completeMapping() -> 发送 complete_map_build 控制命令 (通过 RobotControlChannel)
 * 4. ROS系统停止建图服务并保存地图，通过 report_control_completion 反馈成功
 * 5. Rails 端创建 Map 记录，JS 端接收成功反馈并重置UI状态
 * 
 * 摄像头流程说明：
 * 1. 用户点击"开启摄像头" -> 调用 toggleCameraStream(true) -> 通过 RobotControlChannel 发送指令
 * 2. ROS系统启动摄像头节点，开始采集和处理图像
 * 3. 处理后的图像通过 CameraStreamChannel 实时传输到前端
 * 4. 前端接收图像数据并在指定区域显示
 * 
 * 注意：完成建图是通过控制命令而不是取消任务，因为建图需要正确保存地图数据
 */
export default class extends Controller {
  static targets = [
    // 状态显示
    "statusIndicator", "statusText", "connectionStatus",
    // 传感器数据
    "batteryText", "batteryBar", "positionX", "positionY", "linearVelocity", "angularVelocity",
    // 控制模式按钮
    "resumeBtn", "enableManualBtn", "disableManualBtn", "emergencyAlert",
    // 手动控制面板
    "manualControlPanel", "speedSlider", "speedValue",
    // 建图控制
    "mapName", "mapDescription", "startMappingBtn", "completeMappingBtn",
    "taskStatus", "progressBar", "progressText",
    // 地图预览
    "mapPreview", "mapPreviewPlaceholder",
    // 摄像头相关（简化版，去掉帧率和质量控制）
    "cameraToggleBtn", "cameraVideo", "cameraPlaceholder", "cameraStatus",
    // UI容器
    "notificationContainer"
  ]

  static values = {
    initialRobotStatus: String,
    initialEmergencyStopped: Boolean,
    currentMappingTaskId: Number
  }

  connect() {
    console.log("[RobotControl] Controller connected")
    
    // 初始化状态
    this.connectedToRos = false
    this.currentRobotStatus = this.initialRobotStatusValue
    this.isEmergencyStopped = this.initialEmergencyStoppedValue
    this.currentMappingTaskId = this.currentMappingTaskIdValue
    this.isManualControlEnabled = false
    this.currentSpeed = 0.5
    
    // 摄像头状态（简化版）
    this.isCameraStreaming = false
    this.lastFrameTime = 0
    this.frameCount = 0
    this.actualFps = 0
    
    console.log("[RobotControl] Initial camera streaming state:", this.isCameraStreaming)
    
    // 连接channels
    this._connectChannels()
    
    // 设置事件监听
    this._setupEventListeners()
    
    // 初始化UI状态
    this._updateUI()
    this._updateButtonStates()
    this._updateCameraUI()
    
    console.log("[RobotControl] Controller initialization complete")
  }

  disconnect() {
    console.log("[RobotControl] Controller disconnecting")
    this._removeEventListeners()
    this._disconnectChannels()
  }

  // ==== Channel连接管理 ====
  _connectChannels() {
    RobotTaskChannel.connect()
    RobotControlChannel.connect()
    RobotFeedbackInterface.connect()
    CameraStreamChannel.connect()
  }

  _disconnectChannels() {
    RobotTaskChannel.disconnect()
    RobotControlChannel.disconnect()
    RobotFeedbackInterface.disconnect()
    CameraStreamChannel.disconnect()
  }

  // ==== 事件监听设置 ====
  _setupEventListeners() {
    document.addEventListener("robot-feedback:connected", this._handleFeedbackConnected.bind(this))
    document.addEventListener("robot-feedback:disconnected", this._handleFeedbackDisconnected.bind(this))
    document.addEventListener("robot_status_model_update", this._handleRobotStatusUpdate.bind(this))
    document.addEventListener("robot_state_update", this._handleSensorDataUpdate.bind(this))
    document.addEventListener("task_status_update", this._handleTaskUpdate.bind(this))
    document.addEventListener("robot-task-channel:action_success", this._handleTaskActionSuccess.bind(this))
    document.addEventListener("robot-task-channel:action_error", this._handleTaskActionError.bind(this))
    document.addEventListener("robot-control-channel:command_success", this._handleControlCommandSuccess.bind(this))
    document.addEventListener("robot-control-channel:command_error", this._handleControlCommandError.bind(this))
    
    // 摄像头事件监听
    document.addEventListener("camera-stream:connected", this._handleCameraConnected.bind(this))
    document.addEventListener("camera-stream:disconnected", this._handleCameraDisconnected.bind(this))
    document.addEventListener("camera-stream:frame", this._handleCameraFrame.bind(this))
    document.addEventListener("camera-stream:control_success", this._handleCameraControlSuccess.bind(this))
    document.addEventListener("camera-stream:control_error", this._handleCameraControlError.bind(this))
  }

  _removeEventListeners() {
    document.removeEventListener("robot-feedback:connected", this._handleFeedbackConnected.bind(this))
    document.removeEventListener("robot-feedback:disconnected", this._handleFeedbackDisconnected.bind(this))
    document.removeEventListener("robot_status_model_update", this._handleRobotStatusUpdate.bind(this))
    document.removeEventListener("robot_state_update", this._handleSensorDataUpdate.bind(this))
    document.removeEventListener("task_status_update", this._handleTaskUpdate.bind(this))
    document.removeEventListener("robot-task-channel:action_success", this._handleTaskActionSuccess.bind(this))
    document.removeEventListener("robot-task-channel:action_error", this._handleTaskActionError.bind(this))
    document.removeEventListener("robot-control-channel:command_success", this._handleControlCommandSuccess.bind(this))
    document.removeEventListener("robot-control-channel:command_error", this._handleControlCommandError.bind(this))
    
    // 摄像头事件移除
    document.removeEventListener("camera-stream:connected", this._handleCameraConnected.bind(this))
    document.removeEventListener("camera-stream:disconnected", this._handleCameraDisconnected.bind(this))
    document.removeEventListener("camera-stream:frame", this._handleCameraFrame.bind(this))
    document.removeEventListener("camera-stream:control_success", this._handleCameraControlSuccess.bind(this))
    document.removeEventListener("camera-stream:control_error", this._handleCameraControlError.bind(this))
  }

  // ==== 事件处理器 ====
  _handleFeedbackConnected() {
    this.connectedToRos = true
    this._updateConnectionStatus()
  }

  _handleFeedbackDisconnected() {
    this.connectedToRos = false
    this._updateConnectionStatus()
  }

  _handleRobotStatusUpdate(event) {
    const statusData = event.detail
    console.log("[RobotControl] Robot status update:", statusData)
    
    this.currentRobotStatus = statusData.status
    this.isEmergencyStopped = statusData.is_emergency_stopped
    
    // 处理活动地图信息
    if (statusData.active_map) {
      this.currentActiveMap = statusData.active_map
      this._updateActiveMapDisplay()
    } else if (statusData.active_map === null) {
      this.currentActiveMap = null
      this._updateActiveMapDisplay()
    }
    
    this._updateUI()
    this._updateButtonStates()
  }

  _handleSensorDataUpdate(event) {
    const sensorData = event.detail
    this._updateSensorDisplay(sensorData)
  }

  _handleTaskUpdate(event) {
    const taskData = event.detail
    console.log("[RobotControl] Task update:", taskData)
    
    if (taskData.task_type === 'map_build_auto') {
      this._updateMappingTaskDisplay(taskData)
    }
  }

  _handleTaskActionSuccess(event) {
    const { action, data } = event.detail
    console.log("[RobotControl] Task action success:", action, data)
    
    if (action === "create_task") {
      this._showNotification(`任务已创建: #${data.task_id}`, "success")
      
      if (data.task_type === "map_build_auto") {
        this.currentMappingTaskId = data.task_id
        this._updateMappingButtonStates(true)
      }
    }
  }

  _handleTaskActionError(event) {
    const { action, errors } = event.detail
    console.log("[RobotControl] Task action error:", action, errors)
    
    this._showNotification(`操作失败: ${errors.join(", ")}`, "error")
  }

  _handleControlCommandSuccess(event) {
    const { data } = event.detail
    console.log("[RobotControl] Control command success:", data)
    
    // 处理摄像头控制成功响应
    if (data.enabled !== undefined) {
      // 这是摄像头切换响应
      this.isCameraStreaming = data.enabled
      this._updateCameraUI()
      this._showNotification(data.message || "摄像头控制成功", "success")
    } 
    // 处理手动控制启用/禁用成功
    else if (data.command === 'enable_manual_control') {
      this.isManualControlEnabled = true
      this._updateButtonStates()
      this._showNotification("手动控制模式已启用", "success")
    }
    else if (data.command === 'disable_manual_control') {
      this.isManualControlEnabled = false
      this._updateButtonStates()
      this._showNotification("已返回自主模式", "success")
    }
    else if (data.message) {
      // 其他控制命令响应
      this._showNotification(data.message, "success")
    } else {
      this._showNotification("控制命令执行成功", "success")
    }
  }

  _handleControlCommandError(event) {
    const { command, errors } = event.detail
    console.log("[RobotControl] Control command error:", command, errors)
    
    this._showNotification(`控制失败: ${errors.join(", ")}`, "error")
  }

  _handleCameraConnected() {
    console.log("[RobotControl] Camera stream connected")
    this._updateCameraConnectionStatus(true)
  }

  _handleCameraDisconnected() {
    console.log("[RobotControl] Camera stream disconnected")
    this._updateCameraConnectionStatus(false)
  }

  _handleCameraFrame(event) {
    const frameData = event.detail
    console.log("[RobotControl] Received camera frame:", frameData.frame_id, frameData.data_size, "bytes")
    this._displayCameraFrame(frameData)
    this._updateCameraStats(frameData)
  }

  _handleCameraControlSuccess(event) {
    const { message, enabled } = event.detail
    console.log("[RobotControl] Camera control success:", message, enabled)
    
    // 这个事件来自CameraStreamChannel，但我们主要通过RobotControlChannel处理控制响应
    // 这里作为备用处理
    if (enabled !== undefined) {
      this.isCameraStreaming = enabled
      this._updateCameraUI()
    }
  }

  _handleCameraControlError(event) {
    const { error } = event.detail
    console.log("[RobotControl] Camera control error:", error)
    
    this._showNotification(`摄像头控制失败: ${error}`, "error")
  }

  // ==== UI更新方法 ====
  _updateUI() {
    this._updateStatusDisplay()
    this._updateManualControlMode()
  }

  _updateStatusDisplay() {
    if (this.hasStatusTextTarget) {
      // 急停状态优先显示
      if (this.isEmergencyStopped) {
        this.statusTextTarget.textContent = "紧急停止状态"
      } else {
        this.statusTextTarget.textContent = this._getStatusText(this.currentRobotStatus)
      }
    }
    
    if (this.hasStatusIndicatorTarget) {
      // 急停状态优先显示红色
      if (this.isEmergencyStopped) {
        this.statusIndicatorTarget.className = "w-3 h-3 rounded-full mr-2 bg-red-500 status-pulse"
      } else {
        const colorClass = this._getStatusColor(this.currentRobotStatus)
        this.statusIndicatorTarget.className = `w-3 h-3 rounded-full mr-2 ${colorClass}`
      }
    }
  }

  _updateConnectionStatus() {
    if (this.hasConnectionStatusTarget) {
      if (this.connectedToRos) {
        this.connectionStatusTarget.textContent = "ROS通信正常"
        this.connectionStatusTarget.className = "text-xs text-green-600"
      } else {
        this.connectionStatusTarget.textContent = "ROS通信断开"
        this.connectionStatusTarget.className = "text-xs text-red-600"
      }
    }
  }

  _updateManualControlMode() {
    this.isManualControlEnabled = (this.currentRobotStatus === "manual_control")
    
    // 手动控制面板的显示由_updateButtonStates()控制，这里不重复处理
  }

  _updateButtonStates() {
    // 急停状态提示
    if (this.hasEmergencyAlertTarget) {
      this.emergencyAlertTarget.classList.toggle("hidden", !this.isEmergencyStopped)
    }

    // 恢复运行按钮
    if (this.hasResumeBtnTarget) {
      this.resumeBtnTarget.classList.toggle("hidden", !this.isEmergencyStopped)
    }

    // 手动控制按钮 - 修改逻辑：急停状态下也允许手动控制
    if (this.hasEnableManualBtnTarget) {
      // 在急停状态下或者idle状态下都可以启用手动控制
      const canEnableManual = (this.isEmergencyStopped || this.currentRobotStatus === "idle") && !this.isManualControlEnabled
      this.enableManualBtnTarget.classList.toggle("hidden", !canEnableManual)
    }
    
    if (this.hasDisableManualBtnTarget) {
      this.disableManualBtnTarget.classList.toggle("hidden", !this.isManualControlEnabled)
    }

    // 手动控制面板 - 确保在急停状态下也能显示
    if (this.hasManualControlPanelTarget) {
      this.manualControlPanelTarget.classList.toggle("hidden", !this.isManualControlEnabled)
    }

    // 建图按钮 - 只有在非急停且idle状态下才能建图
    if (this.hasStartMappingBtnTarget) {
      const canStartMapping = this.currentRobotStatus === "idle" && !this.isEmergencyStopped
      this.startMappingBtnTarget.disabled = !canStartMapping
      this.startMappingBtnTarget.classList.toggle("opacity-50", !canStartMapping)
    }
  }

  _updateSensorDisplay(sensorData) {
    // 电池状态
    if (sensorData.battery_level !== undefined) {
      const level = parseFloat(sensorData.battery_level)
      
      if (this.hasBatteryTextTarget) {
        this.batteryTextTarget.textContent = `${level.toFixed(1)}%`
      }
      
      if (this.hasBatteryBarTarget) {
        this.batteryBarTarget.style.width = `${level}%`
        
        // 更新颜色
        this.batteryBarTarget.classList.remove('bg-green-500', 'bg-yellow-500', 'bg-red-500', 'bg-gray-300')
        if (level > 70) {
          this.batteryBarTarget.classList.add('bg-green-500')
        } else if (level > 30) {
          this.batteryBarTarget.classList.add('bg-yellow-500')
        } else {
          this.batteryBarTarget.classList.add('bg-red-500')
        }
      }
    }

    // 位置信息
    if (sensorData.pose) {
      if (sensorData.pose.x !== undefined && this.hasPositionXTarget) {
        this.positionXTarget.textContent = parseFloat(sensorData.pose.x).toFixed(2)
      }
      if (sensorData.pose.y !== undefined && this.hasPositionYTarget) {
        this.positionYTarget.textContent = parseFloat(sensorData.pose.y).toFixed(2)
      }
    }

    // 速度信息
    if (sensorData.velocity) {
      if (sensorData.velocity.linear !== undefined && this.hasLinearVelocityTarget) {
        this.linearVelocityTarget.textContent = parseFloat(sensorData.velocity.linear).toFixed(2)
      }
      if (sensorData.velocity.angular !== undefined && this.hasAngularVelocityTarget) {
        this.angularVelocityTarget.textContent = parseFloat(sensorData.velocity.angular).toFixed(2)
      }
    }
  }

  _updateMappingTaskDisplay(taskData) {
    // 更新任务状态
    if (this.hasTaskStatusTarget) {
      const statusText = this._getTaskStatusText(taskData.status)
      this.taskStatusTarget.textContent = statusText
    }

    // 更新进度
    if (taskData.progress_percentage !== undefined) {
      const progress = Math.max(0, Math.min(100, parseFloat(taskData.progress_percentage)))
      
      if (this.hasProgressBarTarget) {
        this.progressBarTarget.style.width = `${progress}%`
      }
      
      if (this.hasProgressTextTarget) {
        this.progressTextTarget.textContent = `${progress.toFixed(1)}%`
      }
    }

    // 更新地图预览
    if (taskData.status === 'completed' && taskData.result_data && taskData.result_data.map_preview_url) {
      this._showMapPreview(taskData.result_data.map_preview_url)
    }

    // 更新按钮状态 - 任务结束时重置建图UI
    if (['completed', 'failed', 'cancelled'].includes(taskData.status)) {
      console.log("[RobotControl] Mapping task ended with status:", taskData.status)
      this.currentMappingTaskId = null
      this._updateMappingButtonStates(false)
      
      if (taskData.status === 'completed') {
        this._showNotification("建图任务已完成", "success")
      } else if (taskData.status === 'failed') {
        this._showNotification("建图任务失败", "error")
      } else if (taskData.status === 'cancelled') {
        this._showNotification("建图任务已取消", "warning")
      }
    }
  }

  _showMapPreview(previewUrl) {
    if (this.hasMapPreviewTarget && this.hasMapPreviewPlaceholderTarget) {
      this.mapPreviewTarget.src = previewUrl
      this.mapPreviewTarget.classList.remove("hidden")
      this.mapPreviewPlaceholderTarget.classList.add("hidden")
    }
  }

  _updateMappingButtonStates(isMapping) {
    if (this.hasStartMappingBtnTarget) {
      this.startMappingBtnTarget.disabled = isMapping
      this.startMappingBtnTarget.classList.toggle("opacity-50", isMapping)
    }
    
    if (this.hasCompleteMappingBtnTarget) {
      this.completeMappingBtnTarget.disabled = !isMapping
      this.completeMappingBtnTarget.classList.toggle("opacity-50", !isMapping)
    }
  }

  // 摄像头UI更新（简化版）
  _updateCameraUI() {
    console.log("[RobotControl] Updating camera UI, streaming:", this.isCameraStreaming)
    
    if (this.hasCameraToggleBtnTarget) {
      this.cameraToggleBtnTarget.textContent = this.isCameraStreaming ? "关闭摄像头" : "开启摄像头"
      this.cameraToggleBtnTarget.classList.toggle("bg-red-600", this.isCameraStreaming)
      this.cameraToggleBtnTarget.classList.toggle("bg-green-600", !this.isCameraStreaming)
      this.cameraToggleBtnTarget.classList.toggle("hover:bg-red-700", this.isCameraStreaming)
      this.cameraToggleBtnTarget.classList.toggle("hover:bg-green-700", !this.isCameraStreaming)
    }

    if (this.hasCameraVideoTarget && this.hasCameraPlaceholderTarget) {
      this.cameraVideoTarget.classList.toggle("hidden", !this.isCameraStreaming)
      this.cameraPlaceholderTarget.classList.toggle("hidden", this.isCameraStreaming)
    }
  }

  _updateCameraConnectionStatus(connected) {
    if (this.hasCameraStatusTarget) {
      this.cameraStatusTarget.textContent = connected ? "摄像头连接正常" : "摄像头连接断开"
      this.cameraStatusTarget.className = connected ? "text-xs text-green-600" : "text-xs text-red-600"
    }
  }

  _displayCameraFrame(frameData) {
    if (this.hasCameraVideoTarget && this.isCameraStreaming && frameData.data_url) {
      console.log("[RobotControl] Displaying camera frame")
      this.cameraVideoTarget.src = frameData.data_url
    }
  }

  _updateCameraStats(frameData) {
    // 计算实际帧率
    const currentTime = Date.now()
    if (this.lastFrameTime > 0) {
      const timeDiff = (currentTime - this.lastFrameTime) / 1000
      this.actualFps = (this.actualFps * 0.9) + (1 / timeDiff * 0.1) // 平滑计算
    }
    this.lastFrameTime = currentTime
    this.frameCount++

    // 每5秒更新一次统计信息
    if (this.frameCount % 25 === 0) {
      console.log(`[RobotControl] Camera stats: ${this.actualFps.toFixed(1)} FPS, Frame size: ${frameData.data_size} bytes`)
    }
  }

  // ==== 用户操作方法 ====
  
  // 紧急停止
  emergencyStop() {
    console.log("[RobotControl] Emergency stop requested")
    if (RobotControlChannel.subscription) {
      RobotControlChannel.emergencyStop()
      this._showNotification("紧急停止信号已发送", "warning")
    } else {
      this._showNotification("无法连接到机器人控制通道", "error")
    }
  }

  // 恢复运行
  resumeOperation() {
    console.log("[RobotControl] Resume operation requested")
    if (RobotControlChannel.subscription) {
      RobotControlChannel.resumeOperation()
      this._showNotification("恢复运行信号已发送", "info")
    } else {
      this._showNotification("无法连接到机器人控制通道", "error")
    }
  }

  // 启用手动控制
  enableManualControl() {
    console.log("[RobotControl] Enable manual control requested")
    if (RobotControlChannel.subscription) {
      RobotControlChannel.enableManualControl()
      this._showNotification("正在启用手动控制模式...", "info")
    } else {
      this._showNotification("无法连接到机器人控制通道", "error")
    }
  }

  // 禁用手动控制
  disableManualControl() {
    console.log("[RobotControl] Disable manual control requested")
    if (RobotControlChannel.subscription) {
      RobotControlChannel.disableManualControl()
      this._showNotification("正在返回自主模式...", "info")
    } else {
      this._showNotification("无法连接到机器人控制通道", "error")
    }
  }

  // ==== 手动移动控制 ====
  updateSpeedDisplay() {
    if (this.hasSpeedSliderTarget && this.hasSpeedValueTarget) {
      this.currentSpeed = parseFloat(this.speedSliderTarget.value)
      this.speedValueTarget.textContent = this.currentSpeed.toFixed(1)
    }
  }

  moveForward() {
    this._sendMovementCommand(this.currentSpeed, 0)
  }

  moveBackward() {
    this._sendMovementCommand(-this.currentSpeed, 0)
  }

  moveLeft() {
    this._sendMovementCommand(0, this.currentSpeed)
  }

  moveRight() {
    this._sendMovementCommand(0, -this.currentSpeed)
  }

  stopMotion() {
    this._sendMovementCommand(0, 0)
  }

  _sendMovementCommand(linear, angular) {
    if (!this.isManualControlEnabled) {
      this._showNotification("请先启用手动控制模式", "warning")
      return
    }

    // 在急停状态下，允许手动控制移动（这是安全的，因为用户需要明确启用手动控制）
    if (!this.isEmergencyStopped && this.currentRobotStatus !== "manual_control" && this.currentRobotStatus !== "idle") {
      this._showNotification("当前机器人状态不允许手动移动", "warning")
      return
    }

    if (RobotControlChannel.subscription) {
      const command = {
        linear_velocity: linear,
        angular_velocity: angular
      }
      
      console.log("[RobotControl] Sending movement command:", command)
      RobotControlChannel.sendMovementCommand(command)
    } else {
      this._showNotification("无法连接到机器人控制通道", "error")
    }
  }

  // ==== 建图控制 ====
  startMapping() {
    const mapName = this.mapNameTarget.value.trim()
    const mapDesc = this.mapDescriptionTarget.value.trim()

    if (!mapName) {
      this._showNotification("请输入地图名称", "error")
      return
    }

    const params = {
      map_name: mapName,
      description: mapDesc
    }

    console.log("[RobotControl] Creating mapping task:", params)
    
    if (RobotTaskChannel.subscription) {
      RobotTaskChannel.createTask("map_build_auto", params)
      this._showNotification("正在创建建图任务...", "info")
    } else {
      this._showNotification("无法连接到任务通道", "error")
    }
  }

  completeMapping() {
    if (!this.currentMappingTaskId) {
      this._showNotification("没有正在进行的建图任务", "warning")
      return
    }

    const mapName = this.mapNameTarget.value.trim()
    const mapDesc = this.mapDescriptionTarget.value.trim()

    if (!mapName) {
      this._showNotification("请输入地图名称以完成建图", "error")
      return
    }

    if (confirm(`确定要完成当前的建图任务并保存为"${mapName}"吗？`)) {
      console.log("[RobotControl] Completing mapping task with map name:", mapName)
      
      if (RobotControlChannel.subscription) {
        RobotControlChannel.completeMapBuild(mapName, mapDesc)
        this._showNotification("正在完成建图并保存地图...", "info")
      } else {
        this._showNotification("无法连接到控制通道", "error")
      }
    }
  }

  // ==== 摄像头控制（简化版） ====
  toggleCameraStream() {
    const enable = !this.isCameraStreaming
    console.log("[RobotControl] Toggle camera stream:", enable)
    
    if (RobotControlChannel.subscription) {
      RobotControlChannel.toggleCameraStream(enable)
      this._showNotification(`正在${enable ? '开启' : '关闭'}摄像头...`, "info")
      
      // 临时：立即更新状态以便调试（实际应该等待服务器响应）
      setTimeout(() => {
        console.log("[RobotControl] Forcing camera state update for debugging")
        this.isCameraStreaming = enable
        this._updateCameraUI()
      }, 1000)
    } else {
      this._showNotification("无法连接到机器人控制通道", "error")
    }
  }

  // ==== 通知和状态辅助方法 ====
  _showNotification(message, type = "info") {
    if (!this.hasNotificationContainerTarget) {
      console.warn("[RobotControl] No notification container found")
      return
    }

    const notification = document.createElement("div")
    const notificationClass = this._getNotificationClass(type)
    const icon = this._getNotificationIcon(type)
    
    notification.className = `${notificationClass} px-4 py-3 rounded-lg shadow-lg border-l-4 flex items-start space-x-3 transform transition-all duration-300 translate-x-full`
    notification.innerHTML = `
      <div class="flex-shrink-0">
        ${icon}
      </div>
      <div class="flex-1">
        <p class="text-sm font-medium">${message}</p>
      </div>
      <button onclick="this.parentElement.remove()" class="flex-shrink-0 text-gray-400 hover:text-gray-600">
        <svg class="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M6 18L18 6M6 6l12 12"></path>
        </svg>
      </button>
    `

    this.notificationContainerTarget.appendChild(notification)

    // 动画效果
    setTimeout(() => {
      notification.classList.remove("translate-x-full")
    }, 10)

    // 自动移除
    setTimeout(() => {
      if (notification.parentElement) {
        notification.classList.add("translate-x-full")
        setTimeout(() => {
          if (notification.parentElement) {
            notification.remove()
          }
        }, 300)
      }
    }, 5000)
  }

  _getStatusText(status) {
    const statusMap = {
      "idle": "空闲状态",
      "running": "运行中",
      "manual_control": "手动控制",
      "emergency_stopped": "紧急停止",
      "error": "错误状态"
    }
    return statusMap[status] || "未知状态"
  }

  _getStatusColor(status) {
    const colorMap = {
      "idle": "bg-gray-400",
      "running": "bg-green-500",
      "manual_control": "bg-blue-500",
      "emergency_stopped": "bg-red-500",
      "error": "bg-red-600"
    }
    return colorMap[status] || "bg-gray-300"
  }

  _getTaskStatusText(status) {
    const statusMap = {
      "pending": "等待中",
      "in_progress": "执行中",
      "completed": "已完成",
      "failed": "失败",
      "cancelled": "已取消"
    }
    return statusMap[status] || "未知状态"
  }

  _getNotificationClass(type) {
    const classMap = {
      "success": "bg-green-50 border-green-400 text-green-800",
      "error": "bg-red-50 border-red-400 text-red-800",
      "warning": "bg-yellow-50 border-yellow-400 text-yellow-800",
      "info": "bg-blue-50 border-blue-400 text-blue-800"
    }
    return classMap[type] || classMap["info"]
  }

  _getNotificationIcon(type) {
    const iconMap = {
      "success": `<svg class="w-5 h-5 text-green-400" fill="currentColor" viewBox="0 0 20 20"><path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clip-rule="evenodd"></path></svg>`,
      "error": `<svg class="w-5 h-5 text-red-400" fill="currentColor" viewBox="0 0 20 20"><path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clip-rule="evenodd"></path></svg>`,
      "warning": `<svg class="w-5 h-5 text-yellow-400" fill="currentColor" viewBox="0 0 20 20"><path fill-rule="evenodd" d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z" clip-rule="evenodd"></path></svg>`,
      "info": `<svg class="w-5 h-5 text-blue-400" fill="currentColor" viewBox="0 0 20 20"><path fill-rule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7-4a1 1 0 11-2 0 1 1 0 012 0zM9 9a1 1 0 000 2v3a1 1 0 001 1h1a1 1 0 100-2v-3a1 1 0 00-1-1H9z" clip-rule="evenodd"></path></svg>`
    }
    return iconMap[type] || iconMap["info"]
  }

  _updateActiveMapDisplay() {
    if (this.currentActiveMap && this.currentActiveMap.map_image_url) {
      this._showMapPreview(this.currentActiveMap.map_image_url)
      if (this.hasMapPreviewPlaceholderTarget) {
        this.mapPreviewPlaceholderTarget.innerHTML = `
          <div class="text-center">
            <svg class="mx-auto w-8 h-8 mb-2 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z"></path>
            </svg>
            <p class="text-sm text-green-600 font-medium">已加载地图: ${this.currentActiveMap.name}</p>
            <p class="text-xs text-gray-500 mt-1">机器人当前使用此地图进行导航</p>
          </div>
        `
      }
    } else {
      // 没有活动地图或地图没有图片
      if (this.hasMapPreviewTarget) {
        this.mapPreviewTarget.classList.add("hidden")
      }
      if (this.hasMapPreviewPlaceholderTarget) {
        this.mapPreviewPlaceholderTarget.classList.remove("hidden")
        if (this.currentActiveMap && !this.currentActiveMap.map_image_url) {
          this.mapPreviewPlaceholderTarget.innerHTML = `
            <div class="text-center">
              <svg class="mx-auto w-8 h-8 mb-2 text-yellow-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z"></path>
              </svg>
              <p class="text-sm text-yellow-600 font-medium">已加载地图: ${this.currentActiveMap.name}</p>
              <p class="text-xs text-gray-500 mt-1">地图暂无预览图片</p>
            </div>
          `
        } else {
          this.mapPreviewPlaceholderTarget.innerHTML = `
            <svg class="mx-auto w-12 h-12 mb-3 text-gray-300" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path stroke-linecap="round" stroke-linejoin="round" stroke-width="1" d="M4 16l4.586-4.586a2 2 0 012.828 0L16 16m-2-2l1.586-1.586a2 2 0 012.828 0L20 14m-6-6h.01M6 20h12a2 2 0 002-2V6a2 2 0 00-2-2H6a2 2 0 00-2 2v12a2 2 0 002 2z"></path>
            </svg>
            建图完成后将显示地图预览
          `
        }
      }
    }
  }
} 