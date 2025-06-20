# app/channels/robot_feedback_channel.rb
class RobotFeedbackChannel < ApplicationCable::Channel
  def subscribed
    if connection.robot_client
      # ROS WSRB 节点作为 robot_client 连接到此 Channel，以便调用其下的方法
      # logger.info "[RobotFeedbackChannel] Robot client (WSRB) connected and subscribed for sending data TO Rails."
      logger.info "[RobotFeedbackChannel] Robot client (WSRB) connected for calling actions."
      # elsif params[:stream_identifier].present? && params[:stream_identifier].start_with?("map_preview_task_")
      # 前端JS (robot_controller) 想要订阅特定任务的地图预览流
      # stream_name = params[:stream_identifier]
      # stream_from stream_name
      # logger.info "[RobotFeedbackChannel] Client (User: #{connection.current_user&.id}) subscribed to map preview stream: #{stream_name}"
    else
      logger.warn "[RobotFeedbackChannel] Unauthorized or invalid subscription by non-robot client without valid stream_identifier. Rejecting."
      reject
    end
  end

  def unsubscribed
    if connection.robot_client
      RobotStatus.current.update_status_from_ros({ overall_status: "offline", is_emergency_stopped: false })
      RobotStatus.current.update(current_task_id: nil) # 清除当前任务状态
      RobotStatus.current.active_map.deactivate_in_db! if RobotStatus.current.active_map # 停用当前活动地图
      RobotStatus.current.update(active_map: nil) # 清除当前活动地图
      # 机器人客户端断开连接时，更新状态为 offline
    end
    logger.info "[RobotFeedbackChannel] Client (User: #{connection.current_user&.id} or Robot) unsubscribed."
  end

  def update_robot_state(data)
    return unless ensure_robot_client_and_log_action("update_robot_state", data)
    payload = extract_payload(data)
    return unless payload

    # 更新 Rails DB 中的 RobotStatus 模型记录
    # after_save_commit 回调会广播 "robot_status_model_update" 给前端
    RobotStatus.current.update_status_from_ros(payload)

    # 对于实时且频繁更新的传感器数据，直接广播到 "robot_general_updates_channel" 交给前端展示
    raw_sensor_payload = {
      pose: payload[:pose],
      velocity: payload[:velocity],
      battery_level: payload[:battery_level]
    }.compact

    if raw_sensor_payload.any?
      ActionCable.server.broadcast("robot_general_updates_channel", {
        type: "robot_state_update", # 前端 robot_controller.js 监听此类型处理实时传感器数据
        payload: raw_sensor_payload
      })
    end
  end

  # report_map_preview, report_map_saved, update_task_progress, report_task_completion, report_isbn_scan
  # 这些方法的逻辑基本保持不变，因为它们接收的 payload 结构 (由 TM 通过 TaskFeedback 构造并发给 WSRB)
  # 应该与之前 WSRB 直接模拟时发送的 payload 结构一致。
  # 主要确认 payload 中的 task_id, status_from_ros, map_files 等字段名能对应上。

  # update_task_progress(data) - 保持不变 (payload: { task_id, status_from_ros, ... })
  def update_task_progress(data)
    return unless ensure_robot_client_and_log_action("update_task_progress", data)
    payload = extract_payload(data)
    return unless payload && (task = find_task(payload[:task_id]))
    # ... (更新Task的progress_details和status的逻辑不变) ...
    # (请参考之前提供的完整版本)
    status_from_ros = payload[:status_from_ros].to_s.downcase
    new_task_status = case status_from_ros
    when /queued/ then :queued
    when /started|processing/ then :processing
    when /paused/ then :paused
    else :failed
    end
    current_details = task.progress_details || {}
    new_update_entry = { timestamp: Time.current, status_from_ros: status_from_ros, message: payload[:message], progress_percentage: payload[:progress_percentage], intermediate_results: payload[:intermediate_results] }.compact
    current_details["ros_updates"] ||= []
    current_details["ros_updates"] << new_update_entry
    current_details["last_ros_status"] = payload[:status_from_ros]
    current_details["last_ros_message"] = payload[:message]
    current_details["last_ros_progress"] = payload[:progress_percentage]
    task.progress_details = current_details

    if task.status != new_task_status
      if new_task_status == :processing
        task.started_at ||= Time.current
        # RobotStatus.current.assign_task(task)
        # update_status_from_ros 会处理
      end
      task.status = new_task_status
    end
    unless task.save
      log_feedback_event("Failed to save task progress: #{task.errors.full_messages.join(', ')}", { task_id: task.id }, :error)
    else
      log_feedback_event("Task progress updated.", { task_id: task.id, ros_status: payload[:status_from_ros] })
    end
  end

  def report_control_completion(data)
    return unless ensure_robot_client_and_log_action("report_control_completion", data)
    payload = extract_payload(data)
    return unless payload

    final_ros_status_str = payload[:final_status_from_ros].to_s.downcase
    type = payload[:type]
    result_data = payload[:result_data] || {}

    if type == "complete_map_build"
      if final_ros_status_str == "success"
        # 完成地图构建任务
        map = Map.find_by(id: result_data[:map_id])
        if map
          map.update(map_data_url: result_data[:map_data_url])
          if result_data[:map_image].present?
            image_data = Base64.decode64(result_data[:map_image])
            map.map_image.attach(
              io: StringIO.new(image_data),
              filename: "map_#{map.id}_#{Time.current.to_i}.jpeg",
              content_type: "image/*"
            )
          end
          RobotStatus.current.update(current_task_id: nil)
        end
      elsif final_ros_status_str == "failed"
        map = Map.find_by(id: result_data[:map_id])
        map.delete if map # 删除失败的地图
      end
    end
    log_feedback_event(payload[:message] || "Control command completed with status: #{final_ros_status_str}.", { ros_status: final_ros_status_str })
  end

  def report_task_completion(data)
    return unless ensure_robot_client_and_log_action("report_task_completion", data)
    payload = extract_payload(data)
    return unless payload && (task = find_task(payload[:task_id]))

    final_ros_status_str = payload[:final_status_from_ros].to_s.downcase
    new_task_status = case final_ros_status_str
    when /completed|success/ then :completed
    when /failed|error/ then :failed
    when /cancelling/ then :cancelling
    when /cancelled|aborted/ then :cancelled
    else :failed
    end
    task.status = new_task_status
    task.completed_at = Time.current
    current_result_data = task.result_data || {}
    task.result_data = current_result_data.merge(final_ros_status: final_ros_status_str, ros_message: payload[:message], ros_result_data: payload[:result_data]).compact

    RobotStatus.current.update(current_task_id: nil)

    if task.status_completed?
      if task.task_type_load_map?
        # 处理地图加载任务的完成
        map_id = payload[:result_data][:map_id] || task.map_id
        if map_id.present?
          map = Map.find_by(id: map_id)
          if map
            map.activate_in_db!
            # RobotStatus.current.update(active_map: map)
            # update_status_from_ros会处理
          end
        end
      elsif task.task_type_fetch_book_to_transfer?
        # 处理取书任务完成：更新书籍和槽位状态
        book = task.book
        source_slot = task.source_slot
        target_slot = task.target_slot

        if book && source_slot && target_slot
          # 更新书籍位置
          book.update!(current_slot: target_slot)

          # 更新槽位占用状态
          source_slot.update!(is_occupied: false)
          target_slot.update!(is_occupied: true)

          log_feedback_event("Book '#{book.title}' moved from slot #{source_slot.id} to slot #{target_slot.id}.",
                           { task_id: task.id, book_id: book.id, source_slot_id: source_slot.id, target_slot_id: target_slot.id })
        end
      elsif task.task_type_return_book_from_transfer?
        # 处理还书任务完成：更新书籍和槽位状态
        book = task.book
        source_slot = task.source_slot # 中转站槽位
        target_slot = task.target_slot # 最终归位槽位

        if book && source_slot && target_slot
          # 更新书籍位置
          book.update!(current_slot: target_slot, intended_slot: nil)

          # 更新槽位占用状态
          source_slot.update!(is_occupied: false)
          target_slot.update!(is_occupied: true)

          log_feedback_event("Book '#{book.title}' returned from transit slot #{source_slot.id} to final slot #{target_slot.id}.",
                           { task_id: task.id, book_id: book.id, source_slot_id: source_slot.id, target_slot_id: target_slot.id })
        end
      end
    elsif task.status_failed? || task.status_cancelled?
      # 处理任务失败或取消的情况，需要回滚预分配的资源
      if task.task_type_fetch_book_to_transfer?
        book = task.book
        target_slot = task.target_slot

        if book && target_slot
          # 回滚预分配：清除intended_slot，释放target_slot
          book.update!(intended_slot: nil)
          target_slot.update!(is_occupied: false)

          log_feedback_event("Fetch book task failed/cancelled. Rolled back slot allocation for book '#{book.title}'.",
                           { task_id: task.id, book_id: book.id, target_slot_id: target_slot.id }, :warning)
        end
      end
    end


    unless task.save
      log_feedback_event("Failed to save task completion: #{task.errors.full_messages.join(', ')}", { task_id: task.id }, :error)
    else
      log_feedback_event("Task completion reported. Final Rails status: #{task.status}.", { task_id: task.id, ros_status: final_ros_status_str })
    end
  end

  # def report_isbn_scan(data) # 后续再开发
  # return unless ensure_robot_client_and_log_action("report_isbn_scan", data)
  # payload = extract_payload(data)
  # return unless payload && payload[:scanned_isbn].present?
  # # ... (逻辑不变) ...
  # log_feedback_event("ISBN scan received: #{payload[:scanned_isbn]}.", payload.slice(:task_id, :slot_id_scanned, :confidence))
  # if payload[:task_id] && (task = find_task(payload[:task_id]))
  #   current_details = task.progress_details || {}
  #   current_details["isbn_scans"] ||= []
  #   current_details["isbn_scans"] << payload.slice(:scanned_isbn, :confidence, :slot_id_scanned, :image_snippet_url).merge(timestamp: Time.current.iso8601)
  #   task.progress_details = current_details
  #   task.save
  # end
  # ActionCable.server.broadcast("robot_general_updates_channel", { type: "isbn_scan_update", payload: payload })
  # end

  private
  def ensure_robot_client_and_log_action(action_name, data, log_payload: true)
    unless connection.robot_client
      logger.warn "[RobotFeedbackChannel] Action '#{action_name}' invoked by non-robot client. Rejecting."
      return false
    end
    true
  end

  def extract_payload(data_param)
    raw_payload = data_param.is_a?(String) ? JSON.parse(data_param) : data_param
    payload = raw_payload.is_a?(Hash) ? raw_payload.with_indifferent_access.dig("payload") : nil
    unless payload.is_a?(Hash)
      logger.warn "[RobotFeedbackChannel] Invalid 'payload' Hash: #{data_param.inspect}"
      return nil
    end
    payload
  rescue JSON::ParserError => e
    logger.error "[RobotFeedbackChannel] JSON parse error for payload: #{data_param}. Error: #{e.message}"
    nil
  end

  def find_task(task_id)
    task = Task.find_by(id: task_id)
    logger.warn "[RobotFeedbackChannel] Task ID #{task_id} not found." unless task
    task
  end

  def log_feedback_event(message, details_hash = {}, severity = :info)
    full_message = "[ROS Feedback via WSRB] #{message}" # 强调来源
    details_str = details_hash.map { |k, v| "#{k}:#{v}" }.join(", ")
    full_message += " (#{details_str})" if details_hash.present?
    SystemLog.log(:robot, severity, full_message, "RobotFeedbackChannel", details_hash.slice(:task_id))
  end
end
