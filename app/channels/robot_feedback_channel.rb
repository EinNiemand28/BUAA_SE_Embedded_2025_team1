# app/channels/robot_feedback_channel.rb
class RobotFeedbackChannel < ApplicationCable::Channel
  def subscribed
    if connection.robot_client
      # ROS WSRB 节点作为 robot_client 连接到此 Channel，以便调用其下的方法
      logger.info "[RobotFeedbackChannel] Robot client (WSRB) connected and subscribed for sending data TO Rails."
    elsif params[:stream_identifier].present? && params[:stream_identifier].start_with?("map_preview_task_")
      # 前端JS (robot_controller) 想要订阅特定任务的地图预览流
      stream_name = params[:stream_identifier]
      stream_from stream_name
      logger.info "[RobotFeedbackChannel] Client (User: #{connection.current_user&.id}) subscribed to map preview stream: #{stream_name}"
    else
      logger.warn "[RobotFeedbackChannel] Unauthorized or invalid subscription by non-robot client without valid stream_identifier. Rejecting."
      reject
    end
  end

  def unsubscribed
    logger.info "[RobotFeedbackChannel] Client (User: #{connection.current_user&.id} or Robot) unsubscribed."
  end

  # update_robot_state: 由 WSRB 调用，其 payload 来自 TM 发布的 RobotStatusCompressed
  # payload 期望结构: { pose: {x,y,theta}, velocity: {linear,angular}, battery_level,
  #                     overall_status (TM管理的robot_state_str), error_message, is_emergency_stopped }
  def update_robot_state(data)
    return unless ensure_robot_client_and_log_action("update_robot_state", data)
    payload = extract_payload(data)
    return unless payload

    # 更新 Rails DB 中的 RobotStatus 模型记录
    # RobotStatus.update_status_from_ros 会处理字段映射和保存
    # 并且该模型的 after_save_commit 回调会广播 "robot_status_model_update" 给前端
    RobotStatus.current.update_status_from_ros(payload)

    # 此外，对于非常实时的、可能不直接存入 RobotStatus 模型或希望立即广播的传感器数据，
    # 可以在这里再次广播（如果需要与模型回调分开处理或频率不同）
    # 当前 RobotStatus 模型的回调已足够全面，这里主要确保数据被正确传递给模型。
    # 但如果希望分离“原始传感器数据”和“模型状态”，可以在此广播原始传感器部分。
    raw_sensor_payload = {
        pose: payload[:pose],
        velocity: payload[:velocity],
        battery_level: payload[:battery_level] # battery_level 可能也由模型回调处理
    }.compact

    if raw_sensor_payload.any?
        ActionCable.server.broadcast("robot_general_updates_channel", {
          type: "robot_state_update", # 前端 robot_controller.js 监听此类型处理实时传感器数据
          payload: raw_sensor_payload
        })
    end
    # log_feedback_event("General robot state update received from WSRB (originated from TM).",
    #                    payload.except(:pose, :velocity)) # 记录摘要
  end

  # report_map_preview, report_map_saved, update_task_progress, report_task_completion, report_isbn_scan
  # 这些方法的逻辑基本保持不变，因为它们接收的 payload 结构 (由 TM 通过 TaskFeedback 构造并发给 WSRB)
  # 应该与之前 WSRB 直接模拟时发送的 payload 结构一致。
  # 主要确认 payload 中的 task_id, status_from_ros, map_files 等字段名能对应上。

  # report_map_preview(data) - 保持不变 (payload: { task_id, map_image_data_base64, ... })
  def report_map_preview(data)
    return unless ensure_robot_client_and_log_action("report_map_preview", data, log_payload: false)
    payload = extract_payload(data)
    return unless payload && (task = find_task(payload[:task_id])) && payload[:map_image_data_base64]
    ActionCable.server.broadcast("map_preview_task_#{task.id}", {
      type: "map_preview_update", task_id: task.id,
      map_image_data_base64: payload[:map_image_data_base64],
      frontiers_data: payload[:frontiers_data],
      robot_pose_on_preview: payload[:robot_pose]
    })
    log_feedback_event("Map preview received.", { task_id: task.id })
  end

  # report_map_saved(data) - 保持不变 (payload: { task_id, map_files: [...] })
  def report_map_saved(data)
    return unless ensure_robot_client_and_log_action("report_map_saved", data)
    payload = extract_payload(data)
    return unless payload && (task = find_task(payload[:task_id])) && payload[:map_files].present?
    # ... (创建Map记录并附加文件的逻辑不变) ...
    # (请参考之前提供的完整版本中的 attach_map_file 逻辑)
    map_record = nil
    begin
      ActiveRecord::Base.transaction do
        task_params = task.fetch_parameters
        map_record = Map.new(name: task_params["map_name"] || "Map for Task #{task.id}", description: task_params["description"], created_by_user_id: task.user_id)
        (payload[:map_files] || []).each { |file_info| attach_map_file(map_record, file_info["name"], file_info["content_base64_or_url"], task.id) }
        raise StandardError, "Map record missing PGM or YAML after attachments." unless map_record.map_config.attached? && map_record.map_image.attached?
        map_record.save!
        task.map = map_record
        task.result_data = (task.result_data || {}).merge(map_id_in_rails: map_record.id, map_name_in_rails: map_record.name, files_processed: payload[:map_files].map { |mf| mf["name"] })
        task.save!
      end
      log_feedback_event("Map (ID: #{map_record.id}) created.", { task_id: task.id, map_id: map_record.id })
    rescue StandardError => e
      logger.error "[RobotFeedbackChannel] report_map_saved Error for Task #{task.id}: #{e.message}"
      log_feedback_event("Error saving map: #{e.message}", { task_id: task.id }, :error)
    end
  end

  # update_task_progress(data) - 保持不变 (payload: { task_id, status_from_ros, ... })
  def update_task_progress(data)
    return unless ensure_robot_client_and_log_action("update_task_progress", data)
    payload = extract_payload(data)
    return unless payload && (task = find_task(payload[:task_id]))
    # ... (更新Task的progress_details和status的逻辑不变) ...
    # (请参考之前提供的完整版本)
    task.started_at ||= Time.current if task.status_pending?
    task.status = :processing if task.status_pending?
    current_details = task.progress_details || {}
    new_update_entry = { timestamp: Time.current.iso8601, status_from_ros: payload[:status_from_ros], message: payload[:message], progress_percentage: payload[:progress_percentage], intermediate_results: payload[:intermediate_results] }.compact
    current_details["ros_updates"] ||= []
    current_details["ros_updates"] << new_update_entry
    current_details["last_ros_status"] = payload[:status_from_ros]
    current_details["last_ros_message"] = payload[:message]
    current_details["last_ros_progress"] = payload[:progress_percentage]
    task.progress_details = current_details
    if task.type_map_build_auto? && payload[:status_from_ros].to_s.downcase.include?("started")
        RobotStatus.current.assign_task(task)
    end
    unless task.save
      log_feedback_event("Failed to save task progress: #{task.errors.full_messages.join(', ')}", { task_id: task.id }, :error)
    else
      log_feedback_event("Task progress updated.", { task_id: task.id, ros_status: payload[:status_from_ros] })
    end
  end

  # report_task_completion(data) - 保持不变 (payload: { task_id, final_status_from_ros, ... })
  def report_task_completion(data)
    return unless ensure_robot_client_and_log_action("report_task_completion", data)
    payload = extract_payload(data)
    return unless payload && (task = find_task(payload[:task_id]))
    # ... (更新Task的最终状态、完成时间、结果数据的逻辑不变) ...
    # ... (更新RobotStatus清除当前任务的逻辑不变) ...
    # (请参考之前提供的完整版本)
    final_ros_status_str = payload[:final_status_from_ros].to_s.downcase
    new_task_status = case final_ros_status_str
    when /completed|succeeded/ then :completed
    when /failed|error/ then :failed
    when /cancelled/ then :cancelled
    else :failed
    end
    task.status = new_task_status
    task.completed_at = Time.current
    current_result_data = task.result_data || {}
    task.result_data = current_result_data.merge(final_ros_status: final_ros_status_str, ros_message: payload[:message], ros_result_data: payload[:result_data]).compact
    if task.type_map_build_auto? && task.completed?
        RobotStatus.current.complete_mapping!(task)
    else
        RobotStatus.current.clear_current_task!(task)
    end
    unless task.save
      log_feedback_event("Failed to save task completion: #{task.errors.full_messages.join(', ')}", { task_id: task.id }, :error)
    else
      log_feedback_event("Task completion reported. Final Rails status: #{task.status}.", { task_id: task.id, ros_status: final_ros_status_str })
    end
  end

  # report_isbn_scan(data) - 保持不变
  def report_isbn_scan(data)
    return unless ensure_robot_client_and_log_action("report_isbn_scan", data)
    payload = extract_payload(data)
    return unless payload && payload[:scanned_isbn].present?
    # ... (逻辑不变) ...
    log_feedback_event("ISBN scan received: #{payload[:scanned_isbn]}.", payload.slice(:task_id, :slot_id_scanned, :confidence))
    if payload[:task_id] && (task = find_task(payload[:task_id]))
      current_details = task.progress_details || {}
      current_details["isbn_scans"] ||= []
      current_details["isbn_scans"] << payload.slice(:scanned_isbn, :confidence, :slot_id_scanned, :image_snippet_url).merge(timestamp: Time.current.iso8601)
      task.progress_details = current_details
      task.save
    end
    ActionCable.server.broadcast("robot_general_updates_channel", { type: "isbn_scan_update", payload: payload })
  end

  private # 辅助方法保持不变 (ensure_robot_client_and_log_action, extract_payload, find_task, log_feedback_event, attach_map_file)
  # (请参考之前提供的完整版本)
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

  def attach_map_file(map_record, file_name, content_base64_or_url, task_id_for_log) # 保持不变
    return if content_base64_or_url.blank?
    io_object = nil
    is_url = content_base64_or_url.start_with?("http://", "https://", "file://")
    if is_url
      begin io_object = URI.open(content_base64_or_url)
      rescue StandardError => e; logger.error "[RFC] Failed to open URL '#{content_base64_or_url}' for map file '#{file_name}' (Task #{task_id_for_log}): #{e.message}"; return; end
    else
      begin decoded_content = Base64.decode64(content_base64_or_url); io_object = StringIO.new(decoded_content)
      rescue ArgumentError => e; logger.error "[RFC] Failed to decode Base64 for map file '#{file_name}' (Task #{task_id_for_log}): #{e.message}"; return; end
    end
    if file_name.ends_with?(".yaml"); map_record.map_config.attach(io: io_object, filename: file_name, content_type: "application/x-yaml"); logger.info "[RFC] Attached '#{file_name}' as map_config for Task #{task_id_for_log}."
    elsif file_name.ends_with?(".pgm"); map_record.map_image.attach(io: io_object, filename: file_name, content_type: "image/x-portable-graymap"); logger.info "[RFC] Attached '#{file_name}' as map_image for Task #{task_id_for_log}."
    else; logger.warn "[RFC] Unknown map file type '#{file_name}' for Task #{task_id_for_log}. Not attached."; end
  ensure
    io_object.close if io_object && io_object.respond_to?(:close) && !io_object.is_a?(StringIO) && !is_url
  end
end
