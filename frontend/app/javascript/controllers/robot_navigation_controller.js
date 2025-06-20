import { Controller } from "@hotwired/stimulus"
import RobotTaskChannel from "channels/robot_task_channel"

export default class extends Controller {
  static values = { bookshelfId: Number }

  connect() {
    console.log("[RobotNavigation] Controller connected for bookshelf:", this.bookshelfIdValue)
    
    // 确保RobotTaskChannel已连接
    if (!RobotTaskChannel.subscription) {
      RobotTaskChannel.connect()
    }
  }

  navigateToShelf() {
    if (!this.bookshelfIdValue) {
      this._showNotification("书架ID未找到", "error")
      return
    }

    // 检查是否有活动地图
    this._checkActiveMap().then((hasActiveMap) => {
      if (!hasActiveMap) {
        this._showNotification("没有活动地图，无法执行导航任务", "error")
        return
      }

      // 确认操作
      if (!confirm("确定要让机器人导航到这个书架吗？")) {
        return
      }

      console.log("[RobotNavigation] Creating navigation task for bookshelf:", this.bookshelfIdValue)

      // 创建导航任务
      if (RobotTaskChannel.subscription) {
        const params = {
          bookshelf_id: this.bookshelfIdValue
        }
        
        RobotTaskChannel.createTask("navigation_to_point", params)
        this._showNotification("正在创建导航任务...", "info")
        
        // 监听任务创建结果
        this._setupTaskListeners()
      } else {
        this._showNotification("无法连接到机器人系统", "error")
      }
    })
  }

  async _checkActiveMap() {
    try {
      const response = await fetch('/robots/status.json')
      if (response.ok) {
        const data = await response.json()
        return data.active_map_id !== null
      }
      return false
    } catch (error) {
      console.warn("[RobotNavigation] Could not check active map status:", error)
      return true // 假设有地图，让后端处理验证
    }
  }

  _setupTaskListeners() {
    // 监听任务创建成功
    const successHandler = (event) => {
      const { action, data } = event.detail
      if (action === "create_task" && data.task_type === "navigation_to_point") {
        this._showNotification(`导航任务已创建：#${data.task_id}`, "success")
        
        // 不自动跳转，让用户选择是否查看任务详情
        this._showTaskCreatedActions(data.task_id)
        
        // 移除监听器
        this._removeTaskListeners(successHandler, errorHandler)
      }
    }

    const errorHandler = (event) => {
      const { action, errors } = event.detail
      if (action === "create_task") {
        this._showNotification(`创建导航任务失败: ${errors.join(", ")}`, "error")
        
        // 移除监听器
        this._removeTaskListeners(successHandler, errorHandler)
      }
    }

    document.addEventListener("robot-task-channel:action_success", successHandler)
    document.addEventListener("robot-task-channel:action_error", errorHandler)
    
    // 设置超时清理
    setTimeout(() => {
      this._removeTaskListeners(successHandler, errorHandler)
    }, 10000) // 10秒后自动清理监听器
  }

  _removeTaskListeners(successHandler, errorHandler) {
    document.removeEventListener("robot-task-channel:action_success", successHandler)
    document.removeEventListener("robot-task-channel:action_error", errorHandler)
  }

  _showTaskCreatedActions(taskId) {
    const actionDiv = document.createElement("div")
    actionDiv.className = "fixed bottom-4 right-4 z-50 bg-white border border-gray-200 rounded-lg shadow-lg p-4 max-w-sm"
    actionDiv.innerHTML = `
      <div class="flex items-start space-x-3">
        <div class="flex-shrink-0">
          <svg class="w-6 h-6 text-green-500" fill="currentColor" viewBox="0 0 20 20">
            <path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clip-rule="evenodd"></path>
          </svg>
        </div>
        <div class="flex-1">
          <h4 class="text-sm font-medium text-gray-900">导航任务已创建</h4>
          <p class="mt-1 text-sm text-gray-500">任务 #${taskId} 已提交给机器人</p>
          <div class="mt-3 flex space-x-2">
            <a href="/tasks/${taskId}" class="text-xs bg-blue-600 text-white px-3 py-1 rounded-md hover:bg-blue-700">
              查看任务
            </a>
            <a href="/robots" class="text-xs bg-gray-100 text-gray-700 px-3 py-1 rounded-md hover:bg-gray-200">
              机器人状态
            </a>
            <button onclick="this.closest('div').remove()" class="text-xs text-gray-500 hover:text-gray-700">
              关闭
            </button>
          </div>
        </div>
      </div>
    `

    document.body.appendChild(actionDiv)

    // 15秒后自动移除
    setTimeout(() => {
      if (actionDiv.parentElement) {
        actionDiv.remove()
      }
    }, 15000)
  }

  _showNotification(message, type = "info") {
    // 创建简单的通知显示
    const notification = document.createElement("div")
    const notificationClass = this._getNotificationClass(type)
    
    notification.className = `fixed top-4 right-4 z-50 ${notificationClass} px-4 py-3 rounded-lg shadow-lg border-l-4 flex items-center space-x-3 transform transition-all duration-300 translate-x-full max-w-sm`
    notification.innerHTML = `
      <div class="flex-shrink-0">
        ${this._getNotificationIcon(type)}
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

    document.body.appendChild(notification)

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
}