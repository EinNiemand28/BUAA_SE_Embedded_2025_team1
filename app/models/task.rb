# app/models/task.rb
class Task < ApplicationRecord
  # 关联关系
  belongs_to :user # 任务创建者
  belongs_to :book, optional: true # 任务关联的书籍（如果适用）
  belongs_to :source_slot, class_name: "Slot", optional: true # 来源书架槽位
  belongs_to :target_slot, class_name: "Slot", optional: true # 目标书架槽位
  belongs_to :map, optional: true # 任务关联的地图（例如，建图任务创建的地图，或导航任务使用的地图）
  belongs_to :parent_task, class_name: "Task", optional: true # 父任务（用于复杂任务分解）
  has_many :child_tasks, class_name: "Task", foreign_key: "parent_task_id", dependent: :destroy # 子任务
  has_many :task_logs, class_name: "SystemLog", dependent: :destroy # 任务日志

  # 枚举：任务类型
  enum :task_type, [
    :map_build_auto,            # 自动建图
    :navigation_to_point,       # 导航到点
    :fetch_book_to_transfer,    # 取书到中转站
    :return_book_from_transfer, # 从中转站归位书籍
    :inventory_scan_and_relocate, # 库存扫描并重定位错放书籍
    :load_map                   # 加载指定地图到机器人
    # :system_command,            # 执行系统命令 (根据设计稿)
    # :inventory_full_inspection, # 完整的书架巡检（可能作为父任务）
    # :sub_task_return_book,      # 归位单本书籍的子任务
  ], prefix: true

  # 枚举：任务状态
  enum :status, [
    :pending,                   # 待处理
    :queued,                    # 已排队
    :processing,                # 处理中
    :paused,                    # 已暂停
    :completed,                 # 已完成
    :failed,                    # 已失败
    :cancelling,                # 取消中
    :cancelled                  # 已取消
    # :completed_pending_save_ack, # 可选：表示ROS已完成，等待Rails确认地图等资源保存完毕
  ], prefix: true

  # 验证
  validates :task_type, presence: { message: "任务类型不能为空" }
  validates :status, presence: { message: "任务状态不能为空" }
  validates :user_id, presence: { message: "必须关联一个用户" }
  validates :priority, numericality: { only_integer: true, message: "优先级必须是整数" }

  # 回调
  before_create :set_default_values
  # 当状态、进度详情或结果数据发生变化并成功保存后，广播任务更新
  after_save_commit :broadcast_task_update_if_significant_change, if: -> { saved_change_to_status? || saved_change_to_progress_details? || saved_change_to_result_data? }


  # Rails 7+ 对于数据库字段类型为 json 或 jsonb 的，不需要显式 serialize
  # 如果你的数据库字段是 text 类型，并且你想存储 JSON，那么需要 serialize:
  # serialize :progress_details, coder: JSON
  # serialize :result_data, coder: JSON
  # 建议数据库字段使用 jsonb 类型。

  # 实例方法

  # 检查任务是否可以被取消
  def can_be_cancelled?
    status_pending? || status_processing? || status_queued? || status_paused?
  end

  # 将任务参数（Hash）存储到 progress_details 字段中，通常在任务创建时调用
  # 会保留 progress_details 中已有的其他信息（如ros_updates）
  def store_parameters(params_hash)
    current_details = self.progress_details.is_a?(Hash) ? self.progress_details : {}
    case self.task_type.to_sym
    when :load_map
      params_hash["map_id"] = self.map.id || params_hash["map_id"]
      params_hash["map_name"] = self.map&.name || params_hash["map_name"]
      params_hash["map_data_url"] = self.map&.map_data_url || params_hash["map_data_url"]
    when :navigation_to_point
      params_hash["px"] = self.target_point_x || params_hash["target_point_x"]
      params_hash["py"] = self.target_point_y || params_hash["target_point_y"]
      params_hash["oz"] = self.target_orientation_z || params_hash["target_orientation_z"]
    when :fetch_book_to_transfer, :return_book_from_transfer
      params_hash["book_id"] = self.book.id || params_hash["book_id"]
      source = self.source_slot.absolute_coordinates
      target = self.target_slot.absolute_coordinates
      params_hash["gpx"] = source[:x]
      params_hash["gpy"] = source[:y]
      params_hash["gpz"] = source[:z]
      params_hash["goz"] = source[:oz]
      params_hash["ppx"] = target[:x]
      params_hash["ppy"] = target[:y]
      params_hash["ppz"] = target[:z]
      params_hash["poz"] = target[:oz]
    end

    self.progress_details = current_details.merge(parameters: params_hash || {})
  end

  # 从 progress_details 中安全地获取之前存储的参数
  # 返回一个 Hash，如果未存储则返回空 Hash
  def fetch_parameters
    details = self.progress_details
    details.is_a?(Hash) ? (details["parameters"] || {}) : {}
  end

  private

  # 设置任务创建时的默认值
  def set_default_values
    self.status ||= :pending
    self.priority ||= 0
    # 确保 progress_details 和 result_data 初始化为空哈希，以便安全地合并或访问
    self.progress_details = {} if self.progress_details.nil?
    self.result_data = {} if self.result_data.nil?
  end

  # 广播任务更新到 ActionCable Channel
  def broadcast_task_update_if_significant_change
    # 构建需要广播给前端的任务信息 payload
    task_payload = {
      id: self.id,
      type: self.task_type, # 枚举的字符串形式
      status: self.status,  # 枚举的字符串形式
      # 确保 progress_details 和 result_data 是 Hash (如果它们是JSON字符串，先解析)
      progress_details: self.progress_details || {},
      result_data: self.result_data || {},
      map_id: self.map_id,
      user_id: self.user_id, # 前端可能需要根据用户过滤
      created_at: self.created_at,
      updated_at: self.updated_at,
      scheduled_at: self.scheduled_at,
      started_at: self.started_at,
      completed_at: self.completed_at,
      priority: self.priority
      # 可以添加更多前端展示所需的信息，例如关联对象的名称
      # book_title: self.book&.title,
      # source_slot_identifier: self.source_slot&.identifier, # 假设Slot有identifier方法
      # target_slot_identifier: self.target_slot&.identifier,
      # map_name: self.map&.name
    }

    # 广播到订阅了此特定任务ID的流
    ActionCable.server.broadcast("task_update_channel_#{self.id}", { type: "task_update", task: task_payload })

    # TODO：广播一个简化的更新到全局任务流，用于任务列表等场景
    # ActionCable.server.broadcast("task_update_channel_all", {
    #   type: "task_summary_update",
    #   task: { id: self.id, status: self.status, updated_at: self.updated_at }
    # })

    # 记录系统日志
    log_severity = case self.status.to_sym # 确保比较的是 symbol
    when :completed then :info
    when :failed, :cancelled then :warning # cancelled 也可以是 :info
    else :info
    end
    SystemLog.log(
      :task,
      log_severity,
      "任务 ##{self.id} (#{self.task_type}) 更新已广播. 状态: #{self.status}.",
      "TaskModelCallback", # 来源是模型回调
      { task_id: self.id, user_id: self.user_id }
    )
  end
end
