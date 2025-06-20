import { Controller } from "@hotwired/stimulus"
import consumer from "channels/consumer"
import RobotTaskChannel from "channels/robot_task_channel"
import RobotControlChannel from "channels/robot_control_channel"
import RobotFeedbackInterface from "channels/robot_feedback_channel"

export default class extends Controller {
  static targets = [
    // 状态显示
    "statusIndicator", "statusText", "connectionStatus", "emergencyWarning",
    "operationMode", "currentTaskInfo",
    // 传感器数据
    "batteryText", "batteryBar", "positionX", "positionY", "linearVelocity", "angularVelocity",
    // 控制按钮
    "resumeBtn", "startMappingBtn", "navigateBtn", "fetchBookBtn",
    // 地图选择
    "mapSelect",
    // UI容器
    "modalContainer", "notificationContainer"
  ]

  static values = {
    initialRobotStatus: String,
    initialEmergencyStopped: Boolean,
    currentTaskId: Number
  }

  connect() {
    console.log("[RobotDashboard] Controller connected")
    
    // 初始化状态
    this.connectedToRos = false
    this.currentRobotStatus = this.initialRobotStatusValue
    this.isEmergencyStopped = this.initialEmergencyStoppedValue
    
    // 连接channels
    this._connectChannels()
    
    // 设置事件监听
    this._setupEventListeners()
    
    // 初始化UI状态
    this._updateUI()
    this._updateButtonStates()
  }

  disconnect() {
    console.log("[RobotDashboard] Controller disconnecting")
    this._removeEventListeners()
    this._disconnectChannels()
  }

  // ==== Channel连接管理 ====
  _connectChannels() {
    RobotTaskChannel.connect()
    RobotControlChannel.connect()
    RobotFeedbackInterface.connect()
  }

  _disconnectChannels() {
    RobotTaskChannel.disconnect()
    RobotControlChannel.disconnect()
    RobotFeedbackInterface.disconnect()
  }

  // ==== 事件监听设置 ====
  _setupEventListeners() {
    document.addEventListener("robot-feedback:connected", this._handleFeedbackConnected.bind(this))
    document.addEventListener("robot-feedback:disconnected", this._handleFeedbackDisconnected.bind(this))
    document.addEventListener("robot_status_model_update", this._handleRobotStatusUpdate.bind(this))
    document.addEventListener("robot_state_update", this._handleSensorDataUpdate.bind(this))
    document.addEventListener("robot-task-channel:action_success", this._handleTaskActionSuccess.bind(this))
    document.addEventListener("robot-task-channel:action_error", this._handleTaskActionError.bind(this))
  }

  _removeEventListeners() {
    document.removeEventListener("robot-feedback:connected", this._handleFeedbackConnected.bind(this))
    document.removeEventListener("robot-feedback:disconnected", this._handleFeedbackDisconnected.bind(this))
    document.removeEventListener("robot_status_model_update", this._handleRobotStatusUpdate.bind(this))
    document.removeEventListener("robot_state_update", this._handleSensorDataUpdate.bind(this))
    document.removeEventListener("robot-task-channel:action_success", this._handleTaskActionSuccess.bind(this))
    document.removeEventListener("robot-task-channel:action_error", this._handleTaskActionError.bind(this))
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
    console.log("[RobotDashboard] Robot status update:", statusData)
    
    this.currentRobotStatus = statusData.status
    this.isEmergencyStopped = statusData.is_emergency_stopped
    
    this._updateUI()
    this._updateButtonStates()
  }

  _handleSensorDataUpdate(event) {
    const sensorData = event.detail
    this._updateSensorDisplay(sensorData)
  }

  _handleTaskActionSuccess(event) {
    const { action, data } = event.detail
    console.log("[RobotDashboard] Task action success:", action, data)
    
    if (action === "create_task") {
      this._showNotification(`任务已创建: #${data.task_id}`, "success")
    }
  }

  _handleTaskActionError(event) {
    const { action, errors } = event.detail
    console.log("[RobotDashboard] Task action error:", action, errors)
    
    this._showNotification(`操作失败: ${errors.join(", ")}`, "error")
  }

  // ==== UI更新方法 ====
  _updateUI() {
    this._updateStatusDisplay()
    this._updateEmergencyWarning()
    this._updateOperationMode()
  }

  _updateStatusDisplay() {
    if (this.hasStatusTextTarget) {
      this.statusTextTarget.textContent = this._getStatusText(this.currentRobotStatus)
    }
    
    if (this.hasStatusIndicatorTarget) {
      const colorClass = this._getStatusColor(this.currentRobotStatus)
      this.statusIndicatorTarget.className = `w-3 h-3 rounded-full mr-2 ${colorClass}`
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

  _updateEmergencyWarning() {
    if (this.hasEmergencyWarningTarget) {
      this.emergencyWarningTarget.classList.toggle("hidden", !this.isEmergencyStopped)
    }
  }

  _updateOperationMode() {
    if (this.hasOperationModeTarget) {
      let modeText = "未知模式"
      
      if (this.isEmergencyStopped) {
        modeText = "紧急停止"
      } else {
        switch (this.currentRobotStatus) {
          case "idle":
            modeText = "空闲状态"
            break
          case "manual_control":
            modeText = "手动控制"
            break
          case "mapping_auto":
            modeText = "自动建图"
            break
          case "navigating":
            modeText = "导航中"
            break
          case "fetching_book":
            modeText = "取书中"
            break
          case "returning_book":
            modeText = "还书中"
            break
          default:
            modeText = this.currentRobotStatus
        }
      }
      
      this.operationModeTarget.textContent = modeText
    }
  }

  _updateButtonStates() {
    // 恢复运行按钮
    if (this.hasResumeBtnTarget) {
      this.resumeBtnTarget.classList.toggle("hidden", !this.isEmergencyStopped)
    }

    // 任务按钮状态
    const canStartTasks = this.currentRobotStatus === "idle" && !this.isEmergencyStopped
    
    if (this.hasStartMappingBtnTarget) {
      this.startMappingBtnTarget.disabled = !canStartTasks
      this.startMappingBtnTarget.classList.toggle("opacity-50", !canStartTasks)
    }
    
    if (this.hasNavigateBtnTarget) {
      this.navigateBtnTarget.disabled = !canStartTasks
      this.navigateBtnTarget.classList.toggle("opacity-50", !canStartTasks)
    }
    
    if (this.hasFetchBookBtnTarget) {
      this.fetchBookBtnTarget.disabled = !canStartTasks
      this.fetchBookBtnTarget.classList.toggle("opacity-50", !canStartTasks)
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

  // ==== 用户操作方法 ====
  emergencyStop() {
    console.log("[RobotDashboard] Emergency stop requested")
    if (RobotControlChannel.subscription) {
      RobotControlChannel.emergencyStop()
      this._showNotification("紧急停止信号已发送", "warning")
    } else {
      this._showNotification("无法连接到机器人控制通道", "error")
    }
  }

  resumeOperation() {
    console.log("[RobotDashboard] Resume operation requested")
    if (RobotControlChannel.subscription) {
      RobotControlChannel.resumeOperation()
      this._showNotification("恢复运行信号已发送", "info")
    } else {
      this._showNotification("无法连接到机器人控制通道", "error")
    }
  }

  // ==== 模态框显示方法 ====
  showMappingModal() {
    const modal = this._createMappingModal()
    this._showModal(modal)
  }

  showNavigationModal() {
    this._loadBookshelvesData().then(bookshelves => {
      const modal = this._createNavigationModal(bookshelves)
      this._showModal(modal)
    }).catch(error => {
      this._showNotification("加载书架数据失败", "error")
    })
  }

  showFetchBookModal() {
    this._loadBooksData().then(books => {
      const modal = this._createFetchBookModal(books)
      this._showModal(modal)
    }).catch(error => {
      this._showNotification("加载图书数据失败", "error")
    })
  }

  // ==== 模态框创建方法 ====
  _createMappingModal() {
    return `
      <div class="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
        <div class="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
          <div class="px-6 py-4 border-b border-gray-200">
            <h3 class="text-lg font-semibold text-gray-900">创建建图任务</h3>
          </div>
          
          <div class="p-6 space-y-4">
            <div>
              <label class="block text-sm font-medium text-gray-700 mb-2">地图名称 *</label>
              <input type="text" id="mapNameInput" class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-blue-500 focus:border-blue-500" placeholder="输入地图名称...">
            </div>
            
            <div>
              <label class="block text-sm font-medium text-gray-700 mb-2">描述（可选）</label>
              <textarea id="mapDescInput" rows="3" class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-blue-500 focus:border-blue-500" placeholder="描述此地图的用途..."></textarea>
            </div>
          </div>
          
          <div class="px-6 py-4 border-t border-gray-200 flex justify-end space-x-3">
            <button onclick="this.closest('.fixed').remove()" class="px-4 py-2 text-gray-700 border border-gray-300 rounded-lg hover:bg-gray-50">
              取消
            </button>
            <button onclick="window.robotDashboard.createMappingTask()" class="px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700">
              开始建图
            </button>
          </div>
        </div>
      </div>
    `
  }

  _createNavigationModal(bookshelves) {
    const options = bookshelves.map(shelf => 
      `<option value="${shelf.id}">${shelf.name} (${shelf.location})</option>`
    ).join('')

    return `
      <div class="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
        <div class="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
          <div class="px-6 py-4 border-b border-gray-200">
            <h3 class="text-lg font-semibold text-gray-900">导航到书架</h3>
          </div>
          
          <div class="p-6">
            <label class="block text-sm font-medium text-gray-700 mb-2">选择目标书架</label>
            <select id="bookshelfSelect" class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-blue-500 focus:border-blue-500">
              <option value="">请选择...</option>
              ${options}
            </select>
          </div>
          
          <div class="px-6 py-4 border-t border-gray-200 flex justify-end space-x-3">
            <button onclick="this.closest('.fixed').remove()" class="px-4 py-2 text-gray-700 border border-gray-300 rounded-lg hover:bg-gray-50">
              取消
            </button>
            <button onclick="window.robotDashboard.createNavigationTask()" class="px-4 py-2 bg-indigo-600 text-white rounded-lg hover:bg-indigo-700">
              开始导航
            </button>
          </div>
        </div>
      </div>
    `
  }

  _createFetchBookModal(books) {
    const options = books.map(book => 
      `<option value="${book.id}">${book.title} - ${book.isbn}</option>`
    ).join('')

    return `
      <div class="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
        <div class="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
          <div class="px-6 py-4 border-b border-gray-200">
            <h3 class="text-lg font-semibold text-gray-900">取书到中转站</h3>
          </div>
          
          <div class="p-6">
            <label class="block text-sm font-medium text-gray-700 mb-2">选择图书</label>
            <select id="bookSelect" class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-blue-500 focus:border-blue-500">
              <option value="">请选择...</option>
              ${options}
            </select>
          </div>
          
          <div class="px-6 py-4 border-t border-gray-200 flex justify-end space-x-3">
            <button onclick="this.closest('.fixed').remove()" class="px-4 py-2 text-gray-700 border border-gray-300 rounded-lg hover:bg-gray-50">
              取消
            </button>
            <button onclick="window.robotDashboard.createFetchBookTask()" class="px-4 py-2 bg-green-600 text-white rounded-lg hover:bg-green-700">
              开始取书
            </button>
          </div>
        </div>
      </div>
    `
  }

  // ==== 任务创建方法 ====
  createMappingTask() {
    const mapName = document.getElementById('mapNameInput').value.trim()
    const mapDesc = document.getElementById('mapDescInput').value.trim()

    if (!mapName) {
      this._showNotification("请输入地图名称", "error")
      return
    }

    const params = {
      map_name: mapName,
      description: mapDesc
    }

    console.log("[RobotDashboard] Creating mapping task:", params)
    
    if (RobotTaskChannel.subscription) {
      RobotTaskChannel.createTask("map_build_auto", params)
      this._closeModal()
    } else {
      this._showNotification("无法连接到任务通道", "error")
    }
  }

  createNavigationTask() {
    const bookshelfId = document.getElementById('bookshelfSelect').value

    if (!bookshelfId) {
      this._showNotification("请选择目标书架", "error")
      return
    }

    const params = {
      bookshelf_id: parseInt(bookshelfId)
    }

    console.log("[RobotDashboard] Creating navigation task:", params)
    
    if (RobotTaskChannel.subscription) {
      RobotTaskChannel.createTask("navigation_to_point", params)
      this._closeModal()
    } else {
      this._showNotification("无法连接到任务通道", "error")
    }
  }

  createFetchBookTask() {
    const bookId = document.getElementById('bookSelect').value

    if (!bookId) {
      this._showNotification("请选择图书", "error")
      return
    }

    const params = {
      book_id: parseInt(bookId)
    }

    console.log("[RobotDashboard] Creating fetch book task:", params)
    
    if (RobotTaskChannel.subscription) {
      RobotTaskChannel.createTask("fetch_book_to_transfer", params)
      this._closeModal()
    } else {
      this._showNotification("无法连接到任务通道", "error")
    }
  }

  // ==== 地图管理 ====
  loadSelectedMap() {
    const mapId = this.mapSelectTarget.value

    if (!mapId) {
      this._showNotification("请选择地图", "error")
      return
    }

    const params = {
      map_id: parseInt(mapId)
    }

    console.log("[RobotDashboard] Loading map:", params)
    
    if (RobotTaskChannel.subscription) {
      RobotTaskChannel.createTask("load_map", params)
    } else {
      this._showNotification("无法连接到任务通道", "error")
    }
  }

  // ==== 任务取消 ====
  cancelTask(event) {
    const taskId = parseInt(event.params.taskId)
    
    if (confirm("确定要取消这个任务吗？")) {
      console.log("[RobotDashboard] Canceling task:", taskId)
      
      if (RobotControlChannel.subscription) {
        RobotControlChannel.cancelTask({ task_id: taskId })
      } else {
        this._showNotification("无法连接到控制通道", "error")
      }
    }
  }

  // ==== 辅助方法 ====
  _showModal(content) {
    if (this.hasModalContainerTarget) {
      this.modalContainerTarget.innerHTML = content
      // 设置全局引用以便模态框内部调用
      window.robotDashboard = this
    }
  }

  _closeModal() {
    if (this.hasModalContainerTarget) {
      this.modalContainerTarget.innerHTML = ''
    }
  }

  _showNotification(message, type = "info") {
    if (!this.hasNotificationContainerTarget) return

    const notification = document.createElement('div')
    notification.className = `px-4 py-3 rounded-lg shadow-lg transform transition-all duration-300 ${this._getNotificationClass(type)}`
    
    notification.innerHTML = `
      <div class="flex items-start">
        <div class="flex-shrink-0">
          ${this._getNotificationIcon(type)}
        </div>
        <div class="ml-3 flex-1">
          <p class="text-sm font-medium">${message}</p>
        </div>
        <div class="ml-4 flex-shrink-0">
          <button onclick="this.closest('div').remove()" class="inline-flex rounded-md bg-inherit text-gray-400 hover:text-gray-500">
            <svg class="h-5 w-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M6 18L18 6M6 6l12 12"></path>
            </svg>
          </button>
        </div>
      </div>
    `

    this.notificationContainerTarget.appendChild(notification)

    // 自动移除
    setTimeout(() => {
      if (notification.parentNode) {
        notification.remove()
      }
    }, 5000)
  }

  _getStatusText(status) {
    const statusMap = {
      'offline': '离线',
      'idle': '空闲',
      'manual_control': '手动控制',
      'mapping_auto': '自动建图',
      'navigating': '导航中',
      'fetching_book': '取书中',
      'returning_book': '还书中',
      'emergency_stopped': '紧急停止',
      'error': '错误'
    }
    return statusMap[status] || status
  }

  _getStatusColor(status) {
    const colorMap = {
      'offline': 'bg-gray-400',
      'idle': 'bg-green-500',
      'manual_control': 'bg-blue-500',
      'mapping_auto': 'bg-purple-500',
      'navigating': 'bg-indigo-500',
      'fetching_book': 'bg-orange-500',
      'returning_book': 'bg-orange-500',
      'emergency_stopped': 'bg-red-500',
      'error': 'bg-red-500'
    }
    return colorMap[status] || 'bg-gray-400'
  }

  _getNotificationClass(type) {
    const classMap = {
      'info': 'bg-blue-50 border border-blue-200 text-blue-800',
      'success': 'bg-green-50 border border-green-200 text-green-800',
      'warning': 'bg-yellow-50 border border-yellow-200 text-yellow-800',
      'error': 'bg-red-50 border border-red-200 text-red-800'
    }
    return classMap[type] || classMap.info
  }

  _getNotificationIcon(type) {
    const iconMap = {
      'info': '<svg class="h-5 w-5 text-blue-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"></path></svg>',
      'success': '<svg class="h-5 w-5 text-green-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M5 13l4 4L19 7"></path></svg>',
      'warning': '<svg class="h-5 w-5 text-yellow-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z"></path></svg>',
      'error': '<svg class="h-5 w-5 text-red-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M6 18L18 6M6 6l12 12"></path></svg>'
    }
    return iconMap[type] || iconMap.info
  }

  // ==== 数据加载方法 ====
  async _loadBookshelvesData() {
    try {
      const response = await fetch('/api/bookshelves')
      if (!response.ok) throw new Error('Failed to fetch bookshelves')
      const data = await response.json()
      return data.bookshelves
    } catch (error) {
      console.error('Error loading bookshelves:', error)
      throw error
    }
  }

  async _loadBooksData() {
    try {
      const response = await fetch('/books/search?format=json')
      if (!response.ok) throw new Error('Failed to fetch books')
      const data = await response.json()
      return data.books || []
    } catch (error) {
      console.error('Error loading books:', error)
      throw error
    }
  }
} 