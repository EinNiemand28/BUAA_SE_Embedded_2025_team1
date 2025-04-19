class SystemNotificationWorker
  # include Sidekiq::Worker
  # sidekiq_options queue: 'default'

  # def perform
  #   # 生成一条系统通知消息
  #   message = "系统消息: Sidekiq工作正常！时间: #{Time.current.strftime('%Y-%m-%d %H:%M:%S')}"
    
  #   # 获取现有消息数组或创建新数组
  #   messages = Rails.cache.fetch('system_notifications', expires_in: 1.day) { [] }
    
  #   # 添加新消息到数组头部（限制最多保存10条）
  #   messages.unshift({
  #     message: message,
  #     level: 'info',
  #     created_at: Time.current
  #   })
  #   messages = messages.take(10)
    
  #   # 更新缓存
  #   Rails.cache.write('system_notifications', messages, expires_in: 1.day)
    
  #   # 记录到日志
  #   Rails.logger.info "SystemNotificationWorker执行成功: #{message}"
  # end
end
