# app/channels/robot_task_channel.rb
class RobotTaskChannel < ApplicationCable::Channel
  # 这个 Channel 主要由前端（已登录用户）调用，用于创建任务或取消任务。
  # ROS 节点本身不应该订阅这个 Channel 来接收指令，
  # 而是由 Rails 后端在处理完这里的请求后，通过一个特定的流（例如 "ros_comms_channel"）
  # 或者直接向已建立连接的 ROS 客户端发送格式化后的指令。

  def subscribed
    # 确保只有已登录用户可以订阅和调用此 Channel 的方法
    unless connection.current_user
      logger.warn "[RobotTaskChannel] Unauthorized subscription attempt by non-user. Rejecting."
      reject
      return
    end
    # stream_from "robot_task_channel_user_#{connection.current_user.id}" # 可选：如果需要向调用者回传特定消息
    logger.info "[RobotTaskChannel] User #{connection.current_user.id} subscribed."
  end

  def unsubscribed
    logger.info "[RobotTaskChannel] User #{connection.current_user&.id} unsubscribed."
  end

  # 前端调用此方法创建新任务
  def create_task(data)
    user = connection.current_user
    # 已经在 subscribed 中检查了 user 是否存在，但再次确认以防万一
    unless user
      transmit_error("User not authenticated.")
      return
    end

    task_type_str = data.dig("task_type")
    parameters = data.dig("parameters") || {}

    # 权限检查：例如，只有管理员能创建建图任务
    if task_type_str == "map_build_auto" && !user.admin?
      transmit_error("Admin privileges required for MAP_BUILD_AUTO task.")
      return
    end

    # 检查机器人状态是否允许处理此任务
    robot_status = RobotStatus.current
    unless robot_status.can_process_task?(task_type_str&.to_sym)
      transmit_error("Robot is not in a state to process '#{task_type_str}'. Current status: #{robot_status.status}")
      return
    end

    # 验证建图任务参数
    if task_type_str == "map_build_auto"
      map_name = parameters["map_name"]
      if map_name.blank?
        transmit_error("Map name is required for mapping task.")
        return
      end
      # 可选：检查 Map.exists?(name: map_name) 防止重名，如果需要唯一性
    end

    task = user.tasks.new(
      task_type: task_type_str,
      status: :pending, # 初始状态
      priority: parameters.delete("priority") || 0
    )
    task.store_parameters(parameters) # 将原始参数（如 map_name, description）存入 progress_details

    if task.save
      log_task_event(task, "Task created successfully.")

      # 更新 RobotStatus，指派任务 (如果适用，例如对于独占性任务)
      # 对于建图任务，可以在这里就更新 RobotStatus 为 mapping，或等待ROS确认开始
      # robot_status.assign_task(task) if task.type_map_build_auto? # 假设 assign_task 会更新状态为 mapping

      # 构建并向 ROS 广播任务执行指令
      broadcast_task_to_ros("TASK_EXECUTE", task)

      # 向调用者前端回传成功信息和任务详情
      transmit({
        status: "success",
        task_id: task.id,
        task_type: task.task_type,
        initial_status: task.status,
        message: "Task ##{task.id} (#{task.task_type}) created."
      })
    else
      log_task_event(task, "Failed to save Task: #{task.errors.full_messages.join(', ')}", :error_level)
      transmit_error(task.errors.full_messages)
    end
  end

  # 前端调用此方法取消任务
  def cancel_task(data)
    user = connection.current_user
    unless user
      transmit_error("User not authenticated.")
      return
    end

    task_id = data.dig("task_id")
    task = Task.find_by(id: task_id)

    unless task
      transmit_error("Task with ID #{task_id} not found.")
      return
    end

    unless task.user_id == user.id || user.admin?
      transmit_error("Not authorized to cancel this task.")
      return
    end

    if task.can_be_cancelled?
      # 更新任务状态为 cancelling，实际的 "cancelled" 状态应由ROS反馈后设置
      if task.update(status: :cancelling)
        log_task_event(task, "Task cancellation requested.")

        # 向ROS广播取消指令
        broadcast_task_to_ros("TASK_CANCEL", task)

        transmit({
          status: "success",
          task_id: task.id,
          new_status: task.status.to_s,
          message: "Task ##{task.id} cancellation request sent."
        })
      else
        log_task_event(task, "Failed to update task status to cancelling: #{task.errors.full_messages.join(', ')}", :error_level)
        transmit_error(task.errors.full_messages)
      end
    else
      transmit_error("Task ##{task.id} cannot be cancelled (current status: #{task.status}).")
    end
  end

  private

  def broadcast_task_to_ros(command_type, task)
    # 从 task.progress_details 中获取原始参数
    ros_parameters = task.fetch_parameters

    # 为特定任务类型准备额外的参数
    if task.type_load_map? && task.map
        ros_parameters[:map_id_in_rails] = task.map.id
        ros_parameters[:map_name] ||= task.map.name # 如果创建时没给，用关联地图的
        # 确保 map_config_url 和 map_image_url 存在且可访问
        if task.map.map_config.attached? && task.map.map_image.attached?
          base_url = Rails.application.config.action_controller.asset_host || Rails.application.routes.url_helpers.root_url(protocol: "http") # 确保有协议和主机
          ros_parameters[:map_config_url] = Rails.application.routes.url_helpers.rails_blob_url(task.map.map_config, host: base_url)
          ros_parameters[:map_image_url] = Rails.application.routes.url_helpers.rails_blob_url(task.map.map_image, host: base_url)
        else
            logger.error "[RobotTaskChannel] LOAD_MAP Task #{task.id}: Map #{task.map.id} is missing attached config or image."
            # 可以在此处将任务标记为失败，或通知前端
            task.update(status: :failed, result_data: (task.result_data || {}).merge(error: "Map files missing for LOAD_MAP task."))
            return # 不向ROS发送指令
        end
    elsif task.type_navigation_to_point? # 导航任务可能需要当前活动地图的信息
        active_map = RobotStatus.current.active_map
        if active_map
            ros_parameters[:active_map_name_in_rails] = active_map.name
        # ROS可能需要知道当前地图的引用（如ID或URL），这里根据ROS的需求来定
        # ros_parameters[:active_map_reference] = active_map.id # 或者URL
        else
            logger.error "[RobotTaskChannel] NAVIGATION_TO_POINT Task #{task.id}: No active map found."
            task.update(status: :failed, result_data: (task.result_data || {}).merge(error: "No active map for navigation."))
            return # 不向ROS发送指令
        end
    end
    # 对于 MAP_BUILD_AUTO，ros_parameters 已经包含了 map_name 和 description

    instruction_to_ros = {
      command_type: command_type, # "TASK_EXECUTE" or "TASK_CANCEL"
      task: {
        id: task.id,
        type: task.task_type.to_s.upcase, # E.g., "MAP_BUILD_AUTO"
        parameters: ros_parameters
      }
    }
    # 广播到所有已连接的ROS客户端监听的流
    ActionCable.server.broadcast("ros_comms_channel", instruction_to_ros)
    logger.info "[RobotTaskChannel] Broadcasted #{command_type} for Task #{task.id} to 'ros_comms_channel'."
    logger.debug "[RobotTaskChannel] ROS instruction payload: #{instruction_to_ros.to_json}"
  end

  def transmit_error(messages)
    errors_array = Array(messages) # 确保是数组
    logger.warn "[RobotTaskChannel] Transmitting error to client: #{errors_array.join(', ')}"
    transmit({ status: "error", errors: errors_array })
  end

  def log_task_event(task, message, severity = :info)
    full_message = "Task #{task&.id || 'N/A'} (#{task&.task_type || 'N/A'}): #{message}"
    SystemLog.log(:task, severity, full_message,
                  "RobotTaskChannel",
                  { user_id: connection.current_user&.id, task_id: task&.id })
  end
end
