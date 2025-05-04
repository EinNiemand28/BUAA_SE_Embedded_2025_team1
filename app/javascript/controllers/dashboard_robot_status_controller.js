import { Controller } from "@hotwired/stimulus"
import consumer from "channels/consumer"

export default class extends Controller {
  static targets = [ "statusIndicator", "statusText", "batteryBar", "batteryText" ]

  connect() {
    console.log("DashboardRobotStatus 控制器已连接")
    this.statusSubscription = consumer.subscriptions.create("RobotStatusChannel", {
      connected: this._connected.bind(this),
      disconnected: this._disconnected.bind(this),
      received: this._received.bind(this),
    })
  }

  disconnect() {
    console.log("DashboardRobotStatus 控制器已断开")
    if (this.statusSubscription) {
      this.statusSubscription.unsubscribe()
    }
    this._updateStatusIndicator("disconnected") // 断开时也更新状态
  }

  _connected() {
    console.log("[Dashboard] 已连接到 RobotStatusChannel")
    // 初始连接时不一定代表机器人在线，等待数据
  }

  _disconnected() {
    console.log("[Dashboard] 已断开 RobotStatusChannel")
    this._updateStatusIndicator("disconnected")
    // 可以选择清空电量显示
    this.batteryTextTarget.textContent = "--"
    this.batteryBarTarget.style.width = "0%"
    this.batteryBarTarget.className = "bg-gray-400 h-2.5 rounded-full" // 重置颜色
  }

  _received(data) {
    console.log("[Dashboard] 收到机器人状态:", data)

    // 更新机器人状态 (基于 status 字段或心跳逻辑)
    if (data.status === 'idle' || data.status === 'moving' || data.heartbeat) {
      this._updateStatusIndicator("online")
    } else if (data.status === 'low_battery') {
       this._updateStatusIndicator("warning") // 低电量显示警告
    } else {
       this._updateStatusIndicator("offline") // 其他情况视为离线
    }

    // 更新电池电量
    if (data.battery_level !== undefined) {
      const batteryLevel = Math.round(data.battery_level) // 取整显示
      this.batteryTextTarget.textContent = `${batteryLevel}%`
      this.batteryBarTarget.style.width = `${batteryLevel}%`

      // 根据电量改变颜色
      this.batteryBarTarget.classList.remove("bg-yellow-500", "bg-red-500", "bg-green-500", "bg-gray-400") // 先移除所有颜色
      if (batteryLevel > 50) {
        this.batteryBarTarget.classList.add("bg-green-500")
      } else if (batteryLevel > 20) {
        this.batteryBarTarget.classList.add("bg-yellow-500")
      } else {
        this.batteryBarTarget.classList.add("bg-red-500")
      }
    }
  }

  // 辅助方法更新状态指示器
  _updateStatusIndicator(status) {
    const indicator = this.statusIndicatorTarget
    const statusText = this.statusTextTarget

    indicator.classList.remove("bg-green-400", "bg-red-400", "bg-yellow-400", "bg-gray-400")

    switch (status) {
      case "online":
        indicator.classList.add("bg-green-400")
        statusText.textContent = "在线"
        break
      case "offline":
        indicator.classList.add("bg-red-400")
        statusText.textContent = "离线"
        break
      case "warning": // 用于低电量等
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
}
