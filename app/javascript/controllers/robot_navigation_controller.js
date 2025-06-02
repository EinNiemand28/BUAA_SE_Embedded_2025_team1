import { Controller } from "@hotwired/stimulus"
import RobotTaskChannel from "channels/robot_task_channel"

export default class extends Controller {
  static values = { bookshelfId: Number }

  connect() {
    console.log("[RobotNavigation] Controller connected for bookshelf:", this.bookshelfIdValue)
  }

  navigateToShelf() {
    if (!this.bookshelfIdValue) {
      this._showNotification("书架ID未找到", "error")
      return
    }

    // 确认操作
    if (!confirm("确定要让机器人导航到这个书架吗？")) {
      return
    }

    console.log("[RobotNavigation] Creating navigation task for bookshelf:", this.bookshelfIdValue)

    // 获取书架的导航坐标
    this._getBookshelfNavigationCoordinates()
      .then(coordinates => {
        // 创建导航任务
        if (RobotTaskChannel.subscription) {
          const taskParams = {
            target_point_x: coordinates.x,
            target_point_y: coordinates.y,
            target_orientation_z: coordinates.oz,
            bookshelf_id: this.bookshelfIdValue
          }
          
          RobotTaskChannel.createTask("navigation_to_point", taskParams)
          this._showNotification("正在创建导航任务...", "info")
          
          // 监听任务创建结果
          this._setupTaskListeners()
        } else {
          this._showNotification("无法连接到机器人系统", "error")
        }
      })
      .catch(error => {
        console.error("[RobotNavigation] Failed to get navigation coordinates:", error)
        this._showNotification("获取导航坐标失败", "error")
      })
  }

  async _getBookshelfNavigationCoordinates() {
    try {
      const response = await fetch(`/api/bookshelves/${this.bookshelfIdValue}`)
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`)
      }
      const data = await response.json()
      
      // 计算导航坐标（书架前方0.75米处）
      const shelf = data.bookshelf
      const distance = 0.75 // 导航距离
      const orientation = shelf.orientation || 0
      
      // 书架前方位置计算
      const halfWidth = shelf.width / 2.0
      const frontX = shelf.center_x + halfWidth * Math.cos(orientation)
      const frontY = shelf.center_y + halfWidth * Math.sin(orientation)
      
      const navX = frontX + distance * Math.cos(orientation)
      const navY = frontY + distance * Math.sin(orientation)
      const navOz = orientation + Math.PI // 朝向书架
      
      return {
        x: parseFloat(navX.toFixed(4)),
        y: parseFloat(navY.toFixed(4)),
        oz: parseFloat(navOz.toFixed(4))
      }
    } catch (error) {
      console.error("[RobotNavigation] Error fetching bookshelf data:", error)
      throw error
    }
  }

  _setupTaskListeners() {
    // 监听任务创建成功
    const successHandler = (event) => {
      const { action, data } = event.detail
      if (action === "create_task" && data.task_type === "navigation_to_point") {
        this._showNotification(`导航任务已创建：#${data.task_id}`, "success")
        
        // 可选：跳转到任务详情页
        setTimeout(() => {
          window.location.href = `/tasks/${data.task_id}`
        }, 2000)
        
        // 移除监听器
        document.removeEventListener("robot-task-channel:action_success", successHandler)
        document.removeEventListener("robot-task-channel:action_error", errorHandler)
      }
    }

    const errorHandler = (event) => {
      const { action, errors } = event.detail
      if (action === "create_task") {
        this._showNotification(`创建导航任务失败: ${errors.join(", ")}`, "error")
        
        // 移除监听器
        document.removeEventListener("robot-task-channel:action_success", successHandler)
        document.removeEventListener("robot-task-channel:action_error", errorHandler)
      }
    }

    document.addEventListener("robot-task-channel:action_success", successHandler)
    document.addEventListener("robot-task-channel:action_error", errorHandler)
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