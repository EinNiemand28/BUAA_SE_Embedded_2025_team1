import { Controller } from "@hotwired/stimulus"

// 控制侧边栏和移动端菜单的行为
export default class extends Controller {
  static targets = [
    "mobileMenu", 
    "openIcon", 
    "closeIcon", 
    "userMenu", 
    "sidebarContent", 
    "menuText", 
    "menuLabel", 
    "footerInfo",
    "expandIcon",
    "collapseIcon"
  ]

  connect() {
    // 初始化，确保移动端菜单隐藏
    if (this.hasMobileMenuTarget) {
      this.mobileMenuTarget.classList.add('hidden')
    }
    // 确保用户菜单隐藏
    if (this.hasUserMenuTarget) {
      this.userMenuTarget.classList.add('hidden')
    }
    
    // 检查是否为主侧边栏元素
    const isSidebarMain = this.element.classList.contains('border-r')
    
    // 只有主侧边栏元素才需要初始化折叠状态
    if (isSidebarMain) {
      // 检查本地存储中是否保存了侧边栏状态
      const collapsed = localStorage.getItem('sidebarCollapsed') === 'true'
      if (collapsed) {
        this.collapseSidebar()
      }
    } else {
      // 为悬浮按钮初始化正确的图标状态
      const collapsed = localStorage.getItem('sidebarCollapsed') === 'true'
      if (this.hasExpandIconTarget && this.hasCollapseIconTarget) {
        if (collapsed) {
          this.expandIconTarget.classList.remove('hidden')
          this.collapseIconTarget.classList.add('hidden')
        } else {
          this.expandIconTarget.classList.add('hidden')
          this.collapseIconTarget.classList.remove('hidden')
        }
      }
    }
    this.searchQuery = ""
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
  
  toggleSidebar() {
    // 查找主侧边栏元素
    const mainSidebar = document.querySelector('.flex-col.h-full.bg-white.border-r')
    if (!mainSidebar) return
    
    // 获取主侧边栏的控制器
    const mainSidebarController = this.application.getControllerForElementAndIdentifier(mainSidebar, 'sidebar')
    if (!mainSidebarController) return
    
    // 检查当前状态
    const isCollapsed = mainSidebar.dataset.sidebarCollapsed === 'true'
    
    if (isCollapsed) {
      mainSidebarController.expandSidebar()
    } else {
      mainSidebarController.collapseSidebar()
    }
    
    // 同步浮动按钮的图标状态
    if (this.hasExpandIconTarget && this.hasCollapseIconTarget) {
      if (isCollapsed) {
        this.expandIconTarget.classList.add('hidden')
        this.collapseIconTarget.classList.remove('hidden')
      } else {
        this.expandIconTarget.classList.remove('hidden')
        this.collapseIconTarget.classList.add('hidden')
      }
    }
    
    // 保存状态到本地存储
    localStorage.setItem('sidebarCollapsed', !isCollapsed)
  }
  
  collapseSidebar() {
    // 设置状态
    this.element.dataset.sidebarCollapsed = 'true'
    
    // 更新父容器的样式
    const sidebarContainer = this.element.closest('.md\\:w-64')
    if (sidebarContainer) {
      sidebarContainer.classList.remove('md:w-64')
      sidebarContainer.classList.add('md:w-16')
    }
    
    // 更新主内容区域的padding
    const mainContent = document.querySelector('main')
    if (mainContent) {
      mainContent.classList.add('sidebar-collapsed')
      mainContent.style.paddingLeft = 'calc(4rem + 0.75rem)'
    }
    
    // 隐藏文本内容，但保留图标显示
    const collapseText = this.element.querySelector('span.truncate:not(#expand-text)[data-sidebar-target="menuText"]')
    const expandText = this.element.querySelector('#expand-text')
    
    if (collapseText) {
      collapseText.classList.add('hidden')
    }
    
    if (expandText) {
      expandText.classList.remove('hidden')
    }
    
    if (this.hasMenuTextTarget) {
      this.menuTextTargets.forEach(el => {
        // 跳过展开/收起按钮的文本
        if (el.closest('button[data-action="click->sidebar#toggleSidebar"]')) {
          return
        }
        el.classList.add('opacity-0')
        el.classList.add('hidden')
      })
    }
    
    // 隐藏标签文本
    if (this.hasMenuLabelTarget) {
      this.menuLabelTargets.forEach(el => {
        el.classList.add('opacity-0')
        el.classList.add('hidden')
      })
    }
    
    // 隐藏底部信息
    if (this.hasFooterInfoTarget) {
      this.footerInfoTarget.classList.add('opacity-0')
      this.footerInfoTarget.classList.add('hidden')
    }
    
    // 切换按钮图标
    if (this.hasExpandIconTarget && this.hasCollapseIconTarget) {
      this.expandIconTarget.classList.remove('hidden')
      this.collapseIconTarget.classList.add('hidden')
    }
    
    // 居中所有图标
    const icons = this.element.querySelectorAll('svg:not([data-sidebar-target])')
    icons.forEach(icon => {
      icon.classList.add('mx-auto')
      icon.classList.remove('mr-3')
    })
    
    // 更改所有菜单项的间距
    const menuItems = this.element.querySelectorAll('a.group')
    menuItems.forEach(item => {
      item.classList.add('justify-center')
    })
    
    // 调整搜索框
    const searchForm = this.element.querySelector('form[action="/books/search"]')
    if (searchForm) {
      searchForm.classList.add('hidden')
    }
  }
  
  expandSidebar() {
    // 设置状态
    this.element.dataset.sidebarCollapsed = 'false'
    
    // 更新父容器的样式
    const sidebarContainer = this.element.closest('.md\\:w-16')
    if (sidebarContainer) {
      sidebarContainer.classList.remove('md:w-16')
      sidebarContainer.classList.add('md:w-64')
    }
    
    // 更新主内容区域的padding
    const mainContent = document.querySelector('main')
    if (mainContent) {
      mainContent.classList.remove('sidebar-collapsed')
      if (window.innerWidth >= 768) {
        mainContent.style.paddingLeft = 'calc(16rem + 0.75rem)'
      }
    }
    
    // 处理展开/收起按钮的文本
    const collapseText = this.element.querySelector('span.truncate:not(#expand-text)[data-sidebar-target="menuText"]')
    const expandText = this.element.querySelector('#expand-text')
    
    if (collapseText) {
      collapseText.classList.remove('hidden')
    }
    
    if (expandText) {
      expandText.classList.add('hidden')
    }
    
    // 显示文本内容
    if (this.hasMenuTextTarget) {
      this.menuTextTargets.forEach(el => {
        // 跳过展开/收起按钮的文本
        if (el.closest('button[data-action="click->sidebar#toggleSidebar"]')) {
          return
        }
        el.classList.remove('hidden')
        setTimeout(() => {
          el.classList.remove('opacity-0')
        }, 50)
      })
    }
    
    // 显示标签文本
    if (this.hasMenuLabelTarget) {
      this.menuLabelTargets.forEach(el => {
        el.classList.remove('hidden')
        setTimeout(() => {
          el.classList.remove('opacity-0')
        }, 50)
      })
    }
    
    // 显示底部信息
    if (this.hasFooterInfoTarget) {
      this.footerInfoTarget.classList.remove('hidden')
      setTimeout(() => {
        this.footerInfoTarget.classList.remove('opacity-0')
      }, 50)
    }
    
    // 切换按钮图标
    if (this.hasExpandIconTarget && this.hasCollapseIconTarget) {
      this.expandIconTarget.classList.add('hidden')
      this.collapseIconTarget.classList.remove('hidden')
    }
    
    // 恢复所有图标的位置
    const icons = this.element.querySelectorAll('svg:not([data-sidebar-target])')
    icons.forEach(icon => {
      icon.classList.remove('mx-auto')
      icon.classList.add('mr-3')
    })
    
    // 恢复所有菜单项的间距
    const menuItems = this.element.querySelectorAll('a.group')
    menuItems.forEach(item => {
      item.classList.remove('justify-center')
    })
    
    // 恢复搜索框
    const searchForm = this.element.querySelector('form[action="/books/search"]')
    if (searchForm) {
      searchForm.classList.remove('hidden')
    }
  }

  updateSearchQuery(event) {
    this.searchQuery = event.target.value
  }

  submitSearchOnEnter(event) {
    if (event.key === "Enter") {
      event.preventDefault()
      const form = event.target.closest("form")
      if (form) {
        // 如果需要，可以在此处修改表单的 query 参数值
        // 例如：form.querySelector("input[name=query]").value = this.searchQuery;
        form.requestSubmit()
      }
    }
  }
} 