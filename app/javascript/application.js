// Configure your import map in config/importmap.rb. Read more: https://github.com/rails/importmap-rails
import "@hotwired/turbo-rails"
import "controllers"

// 添加全局事件监听器，用于处理点击其他区域时关闭用户菜单
document.addEventListener('click', (event) => {
  const sidebar = document.querySelector('[data-controller="sidebar"]')
  if (sidebar) {
    const controller = Stimulus.getControllerForElementAndIdentifier(sidebar, 'sidebar')
    if (controller) {
      controller.closeUserMenu(event)
    }
  }
}) 