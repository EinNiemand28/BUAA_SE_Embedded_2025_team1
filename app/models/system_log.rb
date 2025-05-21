# app/models/system_log.rb
class SystemLog < ApplicationRecord
  # 关联关系 (都是可选的，因为日志可能不与特定记录关联)
  belongs_to :user, optional: true
  belongs_to :task, optional: true
  belongs_to :book, optional: true
  # 如果需要，可以添加 belongs_to :map, optional: true 等

  # 枚举：日志类型
  enum :log_type, [
    :system,      # 系统级别事件 (如应用启动、配置更改)
    :robot,      # 机器人硬件或ROS节点相关的通用事件/状态
    :user_action, # 用户执行的操作 (如登录、点击按钮发起任务) - 原为 :user
    :task_event,  # 任务生命周期事件 (创建、状态变更、完成) - 原为 :task
    :app_error,   # Rails应用内部错误/异常 - 原为 :error
    :security,    # 安全相关事件 (如登录失败、权限拒绝)
    :map_event    # 地图相关的事件 (创建、激活)
  ], prefix: true

  # 枚举：日志严重程度
  enum :severity, [
    :info,        # 普通信息
    :warning,     # 警告，可能表示潜在问题
    :error,       # 错误，功能可能未按预期工作 - 原为 :error_level
    :critical,    # 严重错误，可能导致系统不稳定或数据丢失
    :debug       # 调试信息 (通常只在开发环境记录)
  ], prefix: true

  # 验证
  validates :log_type, presence: { message: "日志类型不能为空" }
  validates :severity, presence: { message: "严重程度不能为空" }
  validates :message, presence: { message: "日志消息不能为空" }
  validates :source, presence: { message: "日志来源不能为空" } # 例如：哪个Controller, Model, Channel, Job

  # 默认排序
  default_scope { order(created_at: :desc) }

  # 类方法：便捷创建日志条目
  # SystemLog.log(:user_action, :info, "用户登录成功", "SessionsController#create", user_id: user.id)
  # SystemLog.log(:task_event, :error, "任务执行失败: 超时", "Tasks::ProcessorJob", task_id: task.id, details: { timeout: 120 })
  def self.log(log_type_symbol, severity_symbol, message_text, source_text, options = {})
    # 确保传入的是符号
    log_type_val = log_types[log_type_symbol.to_s]
    severity_val = severities[severity_symbol.to_s]

    unless log_type_val
      Rails.logger.error "[SystemLog] 无效的日志类型: #{log_type_symbol}. 消息: #{message_text}"
      log_type_val = log_types[:system] # 默认为系统日志
    end
    unless severity_val
      Rails.logger.error "[SystemLog] 无效的严重级别: #{severity_symbol}. 消息: #{message_text}"
      severity_val = severities[:info] # 默认为信息级别
    end

    # 提取关联ID
    user_id = options.delete(:user_id)
    task_id = options.delete(:task_id)
    book_id = options.delete(:book_id)
    # map_id  = options.delete(:map_id) # 如果添加了map_id字段

    # 任何其他 options 都作为 details 存储 (如果模型有 details 字段)
    # details_json = options.to_json if options.any?

    create!(
      log_type: log_type_val,
      severity: severity_val,
      message: message_text,
      source: source_text,
      user_id: user_id,
      task_id: task_id,
      book_id: book_id,
      # map_id: map_id,
      # details: details_json # 假设有一个 details: :jsonb 字段
    )
  rescue ActiveRecord::RecordInvalid => e
    Rails.logger.error "[SystemLog] 创建系统日志失败: #{e.message}. 原始消息: #{message_text}"
  rescue => e
    Rails.logger.error "[SystemLog] 创建系统日志时发生未知错误: #{e.message}. 原始消息: #{message_text}"
  end
end
