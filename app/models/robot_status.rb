# app/models/robot_status.rb
class RobotStatus < ApplicationRecord
  # 关联关系
  belongs_to :current_task, class_name: "Task", optional: true
  belongs_to :active_map, class_name: "Map", optional: true

  # 枚举：机器人状态
  # 使用数组定义以避免 Rails 7.1+ 的 deprecation warning (如果值是从0开始的连续整数)
  enum :status, [
    :offline,             # 0: 离线/未连接 (WSRB 可能无法连接到TM，或者TM未启动)
    :initializing,        # 1: TaskManager 正在初始化
    :idle,                # 2: 空闲，准备好接收任务或进入手动模式
    :mapping,             # 3: 正在执行自动建图任务
    :navigating,          # 4: 正在执行导航任务
    :fetching_book,       # 5: 正在执行取书任务
    :returning_book,      # 6: 正在执行还书任务
    :scanning,            # 7: 正在执行扫描任务 (如库存盘点)
    :manual_control,      # 8: 手动控制模式 (由用户通过Web界面控制移动)
    :emergency_stopped,   # 9: 紧急停止状态
    :error                # 10: 发生一般性错误 (非急停，但无法正常工作)
    # :paused, # 如果需要暂停状态可以加回来
  ], prefix: true

  # 验证
  validates :status, presence: { message: "机器人状态不能为空" }

  # 回调
  after_save_commit :broadcast_status_update_to_frontend

  # 类方法
  def self.current
    # RobotStatus 通常只有一个记录，代表全局机器人状态
    # first_or_create! 确保总有一个记录，并设置初始安全状态
    # 更好的初始状态可能是 offline，直到 TaskManager 首次报告状态
    find_or_create_by!(id: 1) do |rs| # 固定ID为1，确保是单例
      rs.status ||= :offline
      rs.is_emergency_stopped ||= false # 初始时不是急停
      logger.info "[RobotStatus] Created new RobotStatus record with default values."
    end
  rescue ActiveRecord::RecordNotUnique # 处理并发创建的罕见情况
    find(1)
  end

  # 实例方法

  # 检查机器人是否可以处理指定类型的任务
  def can_process_task?(task_type_symbol)
    task_type_sym = task_type_symbol.to_s.to_sym rescue nil
    return false unless task_type_sym && Task.task_types.key?(task_type_sym)

    # 任何情况下，急停或手动控制模式下都不能接受新的自动任务
    return false if emergency_stopped? || manual_control? || error? # 出错状态下也不应接受新任务

    case task_type_sym
    when :map_build_auto, :load_map # 这些任务可以从 idle 或 offline 状态发起
      (idle? || offline?) && current_task_id.nil?
    when :navigation_to_point, :fetch_book_to_transfer, :return_book_from_transfer, :inventory_scan_and_relocate
      idle? && active_map.present? && current_task_id.nil?
    else # 其他未知或通用任务
      idle? && current_task_id.nil?
    end
  end

  # 由 RobotFeedbackChannel 调用，基于来自 TaskManager (RobotStatusCompressed) 的 payload 更新
  def update_status_from_ros(ros_payload)
    # ros_payload 是一个Hash，例如:
    # { "pose": {"x":..., "y":..., "theta":...}, "velocity": {...}, "battery_level": ...,
    #   "overall_status": "idle", "error_message": null, "is_emergency_stopped": false }

    attrs_to_update = {}
    log_changes = []

    # 1. 处理急停标志 (这个标志由TM管理，具有高优先级)
    if ros_payload.key?(:is_emergency_stopped)
      new_estop_status = ros_payload[:is_emergency_stopped] == true # 确保是布尔值
      if self.is_emergency_stopped != new_estop_status
        attrs_to_update[:is_emergency_stopped] = new_estop_status
        log_changes << "is_emergency_stopped to #{new_estop_status}"
        # 如果急停状态改变，TM的 overall_status 应该是 emergency_stopped 或 idle/manual_control
        # 我们相信TM的 overall_status，但如果TM只发送了 is_emergency_stopped，则相应更新status
        if new_estop_status && self.status != :emergency_stopped
            attrs_to_update[:status] = :emergency_stopped
            # 如果从非急停进入急停，且有当前任务，任务应被TM中断
            self.current_task&.update(status: :failed, result_data: (self.current_task.result_data||{}).merge(failure_reason: "Robot emergency stop from ROS."))
            attrs_to_update[:current_task_id] = nil # 急停时清除当前任务关联
        elsif !new_estop_status && self.status == :emergency_stopped && ros_payload[:overall_status].to_s != "emergency_stopped"
          # 如果急停解除，且TM的overall_status不是急停，则根据overall_status设置（或默认idle）
          # 注意：RESUME_OPERATION 和 ENABLE_MANUAL_CONTROL 也会改变 is_emergency_stopped 和 status
          # 这里主要是处理TM直接通过RobotStatusCompressed反馈的急停状态变化
        end
      end
    end

    # 2. 处理TM报告的 overall_status (robot_state_str from RobotStatusCompressed)
    if ros_payload.key?(:overall_status)
      tm_reported_state_str = ros_payload[:overall_status].to_s.downcase
      # 将TM的状态字符串转换为 RobotStatus 模型的枚举符号
      # 确保TM的状态字符串能映射到我们定义的枚举值
      if RobotStatus.statuses.key?(tm_reported_state_str)
        tm_reported_state_sym = tm_reported_state_str.to_sym
        # 只有当RobotStatus当前不是由一个Rails端活动任务管理的状态时，才接受TM的overall_status覆盖
        # 或者，当TM报告的状态是急停或错误时，它具有更高优先级
        can_be_overridden_by_tm = ![ :mapping, :navigating, :fetching_book, :returning_book, :scanning ].include?(self.status.to_sym)

        if tm_reported_state_sym == :emergency_stopped # 急停状态总是优先
            attrs_to_update[:status] = :emergency_stopped
            attrs_to_update[:is_emergency_stopped] = true # 确保同步
        elsif self.is_emergency_stopped && tm_reported_state_sym != :emergency_stopped
        # 如果硬件急停已解除 (is_emergency_stopped变为false)，TM的overall_status会变为idle或manual
        # 这种情况已由上面 is_emergency_stopped 的逻辑处理，这里不用重复
        elsif tm_reported_state_sym == :error # 错误状态也较优先
            attrs_to_update[:status] = :error
        elsif can_be_overridden_by_tm || self.status.nil? || self.status == :offline # 允许TM覆盖初始/非任务状态
            if self.status.to_s != tm_reported_state_str
                attrs_to_update[:status] = tm_reported_state_sym
                log_changes << "status to #{tm_reported_state_sym} (from TM overall_status)"
            end
        else # 当前Rails端任务状态优先 (如:mapping)，不被TM的 "idle" 覆盖
            rospy.logdebug(f "[{self.node_name}] TM reported overall_status '{tm_reported_state_str}', but Rails status '{self.status}' is task-specific. Not overriding.")
        end
      else
        rospy.logwarn(f "[{self.node_name}] Received unknown overall_status from TM: '{tm_reported_state_str}'. Not updating status.")
      end
    end

    # 3. 处理错误信息
    if ros_payload.key?(:error_message)
      new_error_msg = ros_payload[:error_message].presence # nil if blank
      if self.error_message != new_error_msg
        attrs_to_update[:error_message] = new_error_msg
        log_changes << "error_message to '#{new_error_msg}'"
        # 如果有新错误且当前状态不是急停/错误，则设为错误
        if new_error_msg.present? && ![ :emergency_stopped, :error ].include?((attrs_to_update[:status] || self.status).to_sym)
          attrs_to_update[:status] = :error unless attrs_to_update.key?(:status) && attrs_to_update[:status] == :emergency_stopped
        # 如果错误被清除，且当前是错误状态（非任务导致的），则尝试恢复到idle
        elsif new_error_msg.nil? && self.status == :error && !is_busy_with_task_type_specific_status?
          attrs_to_update[:status] = :idle unless attrs_to_update.key?(:status) && attrs_to_update[:status] == :emergency_stopped
        end
      end
    end

    # 4. 更新活动地图 (如果TM报告了) - 注意：active_map_id 通常由Rails端任务（如LOAD_MAP）成功后设置
    # if ros_payload.key?(:active_map_name) && ros_payload[:active_map_name].present?
    #   map = Map.find_by(name: ros_payload[:active_map_name])
    #   if map && self.active_map_id != map.id
    #     attrs_to_update[:active_map_id] = map.id
    #     log_changes << "active_map_id to #{map.id} (name: #{map.name})"
    #   end
    # end

    # 5. 更新与任务相关的标志 (is_mapping, is_navigating) - 最好由status推断
    # attrs_to_update[:is_mapping] = (attrs_to_update[:status] || self.status).to_sym == :mapping
    # attrs_to_update[:is_navigating] = (attrs_to_update[:status] || self.status).to_sym == :navigating

    if attrs_to_update.any?
      logger.info "[RobotStatus] Updating from ROS: #{attrs_to_update.inspect}"
      self.update(attrs_to_update) # after_save_commit 会广播
    else
      # logger.debug "[RobotStatus] No changes from ROS payload to update."
    end
  end

  # --- 状态转换方法 (由Rails端逻辑调用，通常是响应用户操作或任务生命周期) ---

  # 当一个任务分配给机器人并开始执行时
  def assign_task(task_to_assign)
    return false unless task_to_assign.is_a?(Task)
    # 只有在机器人空闲或处于可以接受此任务的状态时才能分配
    # can_process_task? 应该在调用此方法前检查，但这里可以再确认
    return false if emergency_stopped? || manual_control? # 急停和手动模式不能分配自动任务

    new_robot_status_key = case task_to_assign.task_type.to_sym
    when :map_build_auto then :mapping
    when :navigation_to_point then :navigating
    when :fetch_book_to_transfer then :fetching_book
    when :return_book_from_transfer then :returning_book
    when :inventory_scan_and_relocate then :scanning
    when :load_map then :processing # 加载地图是个过程
    else :processing # 通用任务执行中
    end
    update(current_task: task_to_assign, status: new_robot_status_key)
  end

  # 当建图任务成功完成
  def complete_mapping!(task_just_completed)
    if self.current_task_id == task_just_completed.id && status_mapping?
      update(status: :idle, current_task: nil, error_message: nil)
    end
  end

  # 当任何任务结束 (成功, 失败, 取消)
  def clear_current_task!(task_just_ended)
    if self.current_task_id == task_just_ended.id
      # 任务结束后，机器人应该回到什么状态？
      # 如果是急停中，保持急停。
      # 如果任务失败或TM本身报错，状态为error。
      # 如果任务被取消且无其他错误，回到idle。
      # 如果任务成功完成，回到idle。
      new_status = if status_emergency_stopped?
                     :emergency_stopped
      elsif task_just_ended.status_failed? || status_error?
                     :error
      elsif task_just_ended.status_cancelled? && !status_error?
                     :idle
      else # completed or other
                     :idle
      end
      update(status: new_status, current_task: nil)
    end
  end

  # 紧急停止 (由RobotControlChannel的action或TM反馈触发)
  def emergency_stop!
    # 如果有当前活动任务，它应该被Task Manager中断，并反馈失败状态
    current_active_task = self.current_task
    if current_active_task && current_active_task.processing?
      # Rails端不能直接“中断”TM中的任务，但可以标记任务失败
      # TM在感知到急停后，会中断其当前任务并发送TaskFeedback(failed_estop)
      # 这里主要是更新Rails端的RobotStatus
    end
    update(status: :emergency_stopped, is_emergency_stopped: true, error_message: "紧急停止已激活 @ #{Time.current.to_s(:db)}", current_task: nil)
  end

  # 从急停恢复到空闲 (由RobotControlChannel的action或TM反馈触发)
  def resume_from_emergency_stop!
    if status_emergency_stopped?
      update(status: :idle, is_emergency_stopped: false, error_message: nil)
    else
      logger.warn "[RobotStatus] Attempted to resume, but not in emergency_stopped state."
      false
    end
  end

  # 从急停进入手动控制模式
  def enable_manual_control!
    if status_emergency_stopped?
      update(status: :manual_control, is_emergency_stopped: false, error_message: nil)
    else
      logger.warn "[RobotStatus] Attempted to enable manual control, but not in emergency_stopped state."
      false
    end
  end

  # 从手动控制模式返回空闲
  def disable_manual_control!
    if status_manual_control?
      update(status: :idle, error_message: nil) # is_emergency_stopped 应该已经是 false
    else
      logger.warn "[RobotStatus] Attempted to disable manual control, but not in manual_control state."
      false
    end
  end

  # (可选) 设置活动地图，通常在 :load_map 任务成功完成后由 RobotFeedbackChannel 调用
  def set_active_map_after_load(map_record)
    if map_record.is_a?(Map)
        update(active_map: map_record)
    end
  end

  private

  def is_busy_with_task_type_specific_status?
    [ :mapping, :navigating, :fetching_book, :returning_book, :scanning ].include?(self.status.to_sym)
  end

  def broadcast_status_update_to_frontend
    status_payload = {
      id: self.id,
      status: self.status.to_s,
      status_text: I18n.t("robots.status.#{self.status}", default: self.status.to_s.humanize),
      is_emergency_stopped: self.is_emergency_stopped, # 从数据库读取
      is_mapping: self.status_mapping?,             # 根据当前status推断
      is_navigating: self.status_navigating?,       # 根据当前status推断
      is_manual_control: self.status_manual_control?, # 新增
      current_task_id: self.current_task_id,
      current_task_type: self.current_task&.task_type, # 包含任务类型
      active_map_id: self.active_map_id,
      active_map_name: self.active_map&.name,
      error_message: self.error_message,
      # battery_level: self.try(:battery_level) || "N/A", # 如果模型有此字段，确保它被更新
      updated_at: self.updated_at.iso8601 # 使用ISO8601格式
    }

    ActionCable.server.broadcast("robot_general_updates_channel", {
      type: "robot_status_model_update",
      payload: status_payload
    })
    # logger.debug "[RobotStatus] Broadcasted model update: #{status_payload.except(:active_map_name)}"
  end
end
