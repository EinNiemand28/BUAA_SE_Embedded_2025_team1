import { Controller } from "@hotwired/stimulus"

// 控制下拉菜单组件的行为
export default class extends Controller {
  static targets = ["menu"]

  connect() {
    // 在控制器连接时添加全局点击事件监听
    this.clickHandler = this.handleOutsideClick.bind(this)
    document.addEventListener("click", this.clickHandler)
  }

  disconnect() {
    // 在控制器断开连接时移除全局点击事件监听
    document.removeEventListener("click", this.clickHandler)
  }

  toggle(event) {
    // 阻止冒泡，防止立即触发outsideClick
    event.stopPropagation()
    // 切换菜单显示状态
    this.menuTarget.classList.toggle("hidden")
  }

  hide() {
    // 隐藏菜单
    this.menuTarget.classList.add("hidden")
  }

  handleOutsideClick(event) {
    // 如果点击的不是控制器元素或其子元素，则隐藏菜单
    if (!this.element.contains(event.target)) {
      this.hide()
    }
  }
} 