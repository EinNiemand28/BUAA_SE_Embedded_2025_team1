import { Controller } from "@hotwired/stimulus"

// 控制侧边栏和移动端菜单的行为
export default class extends Controller {
  static targets = ["mobileMenu", "openIcon", "closeIcon", "userMenu"]

  connect() {
    // 初始化，确保移动端菜单隐藏
    if (this.hasMobileMenuTarget) {
      this.mobileMenuTarget.classList.add('hidden')
    }
    // 确保用户菜单隐藏
    if (this.hasUserMenuTarget) {
      this.userMenuTarget.classList.add('hidden')
    }
  }

  toggleMobileMenu() {
    // 切换移动端菜单的显示/隐藏
    if (this.hasMobileMenuTarget) {
      this.mobileMenuTarget.classList.toggle('hidden')
      
      // 切换图标
      if (this.hasOpenIconTarget && this.hasCloseIconTarget) {
        this.openIconTarget.classList.toggle('hidden')
        this.closeIconTarget.classList.toggle('hidden')
      }
    }
  }

  toggleUserMenu() {
    // 切换用户菜单的显示/隐藏
    if (this.hasUserMenuTarget) {
      this.userMenuTarget.classList.toggle('hidden')
    }
  }

  closeUserMenu(event) {
    // 当点击页面其他地方时关闭用户菜单
    if (this.hasUserMenuTarget && !event.target.closest('#user-menu-button')) {
      this.userMenuTarget.classList.add('hidden')
    }
  }
} 