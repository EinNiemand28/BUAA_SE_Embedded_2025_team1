import { Controller } from "@hotwired/stimulus"
import { Turbo } from "@hotwired/turbo-rails"

export default class extends Controller {
  static targets = ["modal", "overlay", "bookshelfSelect", "slotSelect", "loadingIndicator", "errorMessage", "placementType"]
  static values = { bookId: Number }

  connect() {
    // 控制器连接，无需日志
  }

  // 打开模态框
  openModal(event) {
    if (event) event.preventDefault()
    
    try {
      // 检查目标是否存在
      if (!this.hasModalTarget || !this.hasOverlayTarget) return
      
      // 显示模态框和遮罩 - 使用 show 类
      this.modalTarget.classList.add("show")
      this.overlayTarget.classList.add("show")
      
      // 禁止背景滚动
      document.body.style.overflow = "hidden"
      
      // 重置表单
      this.resetForm()
    } catch (error) {
      console.error("Error opening modal:", error)
  }
  }

  // 关闭模态框
  closeModal(event) {
    if (event) event.preventDefault()
    
    try {
      // 隐藏模态框和遮罩 - 移除 show 类
      this.modalTarget.classList.remove("show")
      this.overlayTarget.classList.remove("show")
      
      // 恢复背景滚动
      document.body.style.overflow = ""
    } catch (error) {
      console.error("Error closing modal:", error)
    }
  }

  // 重置表单状态
  resetForm() {
    try {
      // 重置选择器
    this.bookshelfSelectTarget.value = ""
    this.slotSelectTarget.innerHTML = '<option value="">-- 请先选择书架 --</option>'
    this.slotSelectTarget.disabled = true
      
      // 隐藏加载和错误提示
      this.loadingIndicatorTarget.classList.add("hidden")
      this.errorMessageTarget.classList.add("hidden")
      this.errorMessageTarget.textContent = ""
      
      // 设置默认分配类型
      this.placementTypeTarget.value = "current"
    } catch (error) {
      console.error("Error resetting form:", error)
    }
  }

  // 加载书架的插槽
  async loadSlots() {
    const bookshelfId = this.bookshelfSelectTarget.value
    
    // 重置插槽选择器
    this.slotSelectTarget.innerHTML = '<option value="">-- 请先选择书架 --</option>'
    this.slotSelectTarget.disabled = true
    
    // 隐藏错误信息
    this.errorMessageTarget.classList.add("hidden")

    if (!bookshelfId) return

    // 显示加载指示器
      this.loadingIndicatorTarget.classList.remove("hidden")

    try {
      const response = await fetch(`/bookshelves/${bookshelfId}/slots`)
      if (!response.ok) {
        throw new Error(`加载插槽信息失败: ${response.statusText}`)
      }
      
      const slots = await response.json()

      if (slots.length === 0) {
         this.slotSelectTarget.innerHTML = '<option value="">该书架无可用插槽</option>'
      } else {
        this.slotSelectTarget.innerHTML = '<option value="">-- 请选择插槽 --</option>'
        
        slots.forEach(slot => {
          const occupiedText = slot.is_occupied ? ` (占用: ${slot.current_book_title || '未知'})` : ""
          const option = document.createElement("option")
          option.value = slot.id
          option.textContent = `L${slot.level + 1} R${slot.row + 1}${occupiedText}`
          // 只有当前书籍占用的槽位 或 未被占用的槽位 才能被选择
          option.disabled = slot.is_occupied && slot.current_book_id !== this.bookIdValue
          this.slotSelectTarget.appendChild(option)
        })
        
        this.slotSelectTarget.disabled = false
      }
    } catch (error) {
      console.error("加载插槽失败:", error)
      this.errorMessageTarget.textContent = `加载插槽失败: ${error.message}`
      this.errorMessageTarget.classList.remove("hidden")
    } finally {
        this.loadingIndicatorTarget.classList.add("hidden")
    }
  }

  // 分配插槽
  async assignSlot(event) {
    event.preventDefault()
    
    const slotId = this.slotSelectTarget.value
    const placementType = this.placementTypeTarget.value
    
    if (!slotId) {
      alert("请选择一个插槽！")
      return
    }

    this.errorMessageTarget.classList.add("hidden")

    try {
      const body = {}
      if (placementType === 'current') {
        body.book = { current_slot_id: slotId }
      } else if (placementType === 'intended') {
        body.book = { intended_slot_id: slotId }
      }
      
      const response = await fetch(`/books/${this.bookIdValue}`, {
        method: 'PATCH',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json',
          'X-CSRF-Token': document.querySelector("meta[name='csrf-token']")?.content
        },
        body: JSON.stringify(body)
      })

      if (!response.ok) {
        let errorMessage = `服务器错误: ${response.status}`
        try {
           const errorData = await response.json()
           if (errorData.errors) {
            errorMessage = Object.entries(errorData.errors)
              .map(([field, messages]) => `${field}: ${messages.join(', ')}`)
              .join('; ')
           } else if (errorData.error) {
            errorMessage = errorData.error
           }
        } catch (e) { 
          // 如果响应体不是 JSON 或解析失败，则使用状态文本
          errorMessage = `服务器错误: ${response.statusText || response.status}`
        }
        throw new Error(errorMessage)
      }
      
      this.closeModal()
      Turbo.visit(window.location.href, { action: "replace" })
      
    } catch (error) {
      console.error("分配插槽失败:", error)
      this.errorMessageTarget.textContent = `分配失败: ${error.message}`
      this.errorMessageTarget.classList.remove("hidden")
      }
    }
  
  // 清除插槽分配
  async clearPlacement(event) {
    event.preventDefault()
    
    const placementType = this.placementTypeTarget.value
    const fieldToClear = placementType === 'current' ? 'current_slot_id' : 'intended_slot_id'
    const confirmMessage = placementType === 'current' ? 
        "确定要清除这本书的当前实际位置吗？" : 
        "确定要清除这本书的预定位置吗？"
        
    if (!confirm(confirmMessage)) {
      return
    }
    
    this.errorMessageTarget.classList.add("hidden")
    
    try {
      const body = { book: { [fieldToClear]: null } }
            
      const response = await fetch(`/books/${this.bookIdValue}`, {
        method: 'PATCH',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json',
          'X-CSRF-Token': document.querySelector("meta[name='csrf-token']")?.content
        },
        body: JSON.stringify(body)
      })

      if (!response.ok) {
        let errorMessage = `服务器错误: ${response.status}`
         try {
           const errorData = await response.json()
            if (errorData.errors) {
            errorMessage = Object.entries(errorData.errors)
              .map(([field, messages]) => `${field}: ${messages.join(', ')}`)
              .join('; ')
           } else if (errorData.error) {
            errorMessage = errorData.error
           }
        } catch (e) {
          errorMessage = `服务器错误: ${response.statusText || response.status}`
        }
        throw new Error(errorMessage)
      }
      
      this.closeModal()
      Turbo.visit(window.location.href, { action: "replace" })
      
    } catch (error) {
      console.error("清除位置失败:", error)
      this.errorMessageTarget.textContent = `清除位置失败: ${error.message}`
      this.errorMessageTarget.classList.remove("hidden")
    }
  }
} 