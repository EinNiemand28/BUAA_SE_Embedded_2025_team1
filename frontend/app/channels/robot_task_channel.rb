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
    if [ "map_build_auto", "map_build_manual", "load_map" ].include?(task_type_str)
      unless user.admin?
        transmit_error("Admin privileges required for #{task_type_str.upcase} task.")
        return
      end
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
      if Map.exists?(name: map_name)
        transmit_error("Map with name '#{map_name}' already exists. Please choose a different name.")
        return
      end
    end

    if task_type_str == "load_map"
      map_id = parameters["map_id"]
      map = Map.find_by(id: map_id)
      if map.nil? || map.map_data_url.blank?
        transmit_error("Map not found or incomplete.")
        return
      end
    end

    # 验证导航任务参数
    if task_type_str == "navigation_to_point"
      unless RobotStatus.current.active_map
        transmit_error("No active map available for navigation.")
        return
      end

      bookshelf_id = parameters["bookshelf_id"]
      if bookshelf_id.blank?
        transmit_error("Bookshelf ID is required for navigation task.")
        return
      end

      bookshelf = Bookshelf.find_by(id: bookshelf_id)
      if bookshelf.nil?
        transmit_error("Bookshelf not found.")
        return
      end

      # 计算导航目标坐标
      nav_coords = bookshelf.navigation_coordinates
      parameters.merge!(
        "target_point_x" => nav_coords[:x],
        "target_point_y" => nav_coords[:y],
        "target_point_z" => nav_coords[:z],
        "target_orientation_z" => nav_coords[:oz],
        "bookshelf_name" => bookshelf.name
      )
    end

    # 验证取书任务参数
    if task_type_str == "fetch_book_to_transfer"
      unless RobotStatus.current.active_map
        transmit_error("No active map available for fetch book task.")
        return
      end

      book_id = parameters["book_id"]
      if book_id.blank?
        transmit_error("Book ID is required for fetch book task.")
        return
      end

      book = Book.find_by(id: book_id)
      if book.nil?
        transmit_error("Book not found.")
        return
      end

      # 检查书籍是否有当前位置
      unless book.current_slot
        transmit_error("Book has no current slot location.")
        return
      end

      # 查找中转站的空闲槽位
      transit_station = Bookshelf.transit_stations.first
      unless transit_station
        transmit_error("No transit station available.")
        return
      end

      available_slot = transit_station.slots.where(is_occupied: false).order(relative_z: :desc).first
      unless available_slot
        transmit_error("No available slots in transit station.")
        return
      end

      # 预分配槽位
      begin
        ActiveRecord::Base.transaction do
          available_slot.update!(is_occupied: true)
          book.update!(intended_slot: available_slot)
        end
      rescue ActiveRecord::RecordInvalid => e
        transmit_error("Failed to allocate slot for book: #{e.message}")
        return
      end

      # 获取源位置和目标位置坐标
      source_coords = book.current_slot.absolute_coordinates
      target_coords = available_slot.absolute_coordinates

      parameters.merge!(
        "source_slot_id" => book.current_slot.id,
        "target_slot_id" => available_slot.id,
        "source_x" => source_coords[:x],
        "source_y" => source_coords[:y],
        "source_z" => source_coords[:z],
        "source_oz" => source_coords[:oz],
        "target_x" => target_coords[:x],
        "target_y" => target_coords[:y],
        "target_z" => target_coords[:z],
        "target_oz" => target_coords[:oz],
        "book_title" => book.title,
        "book_isbn" => book.isbn
      )
    end

    # 验证还书任务参数
    if task_type_str == "return_book_from_transfer"
      unless RobotStatus.current.active_map
        transmit_error("No active map available for return book task.")
        return
      end

      book_id = parameters["book_id"]
      if book_id.blank?
        transmit_error("Book ID is required for return book task.")
        return
      end

      book = Book.find_by(id: book_id)
      if book.nil?
        transmit_error("Book not found.")
        return
      end

      # 检查书籍是否在中转站
      unless book.current_slot&.bookshelf&.is_transit_station?
        transmit_error("Book is not currently in transit station.")
        return
      end

      # 检查是否有预定的目标位置
      unless book.intended_slot
        transmit_error("Book has no intended return location.")
        return
      end

      # 获取源位置和目标位置坐标
      source_coords = book.current_slot.absolute_coordinates
      target_coords = book.intended_slot.absolute_coordinates

      parameters.merge!(
        "source_slot_id" => book.current_slot.id,
        "target_slot_id" => book.intended_slot.id,
        "source_x" => source_coords[:x],
        "source_y" => source_coords[:y],
        "source_z" => source_coords[:z],
        "source_oz" => source_coords[:oz],
        "target_x" => target_coords[:x],
        "target_y" => target_coords[:y],
        "target_z" => target_coords[:z],
        "target_oz" => target_coords[:oz],
        "book_title" => book.title,
        "book_isbn" => book.isbn
      )
    end

    task = user.tasks.build(
      task_type: task_type_str,
      status: :pending, # 初始状态
      priority: parameters.delete("priority") || 0,
      book_id: parameters.delete("book_id") || nil,
      source_slot_id: parameters.delete("source_slot_id") || nil,
      target_slot_id: parameters.delete("target_slot_id") || nil,
      target_point_x: parameters.delete("target_point_x") || nil,
      target_point_y: parameters.delete("target_point_y") || nil,
      target_point_z: parameters.delete("target_point_z") || nil,
      target_orientation_z: parameters.delete("target_orientation_z") || nil,
      map_id: parameters.delete("map_id") || nil,
      user_id: user.id,
      scheduled_at: Time.current,
      progress_details: {},
      result_data: {}
    )
    if RobotStatus.current.active_map && !task.task_type_map_build_auto? && !task.task_type_load_map?
      task.map = RobotStatus.current.active_map
    end
    task.store_parameters(parameters) # 将原始参数（如 map_name, description）存入 progress_details

    task.status ||= :pending # 确保状态初始化为 pending

    if task.save
      log_task_event(task, "Task created successfully.")

      # 更新 RobotStatus，指派任务 (如果适用，例如对于独占性任务)
      # 对于建图任务，可以在这里就更新 RobotStatus 为 mapping，或等待ROS确认开始
      # robot_status.assign_task(task) if task.task_type_map_build_auto? # 假设 assign_task 会更新状态为 mapping

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
  # 取消任务是即时的操作, 交由 Control_Channel 处理

  private

  def broadcast_task_to_ros(command_type, task)
    # 从 task.progress_details 中获取原始参数
    ros_parameters = task.fetch_parameters

    # 除非建图或者加载地图任务，否则需要确保有一个活动地图
    unless task.task_type_map_build_auto? || task.task_type_load_map?
      unless RobotStatus.current.active_map
          logger.error "[RobotTaskChannel] NAVIGATION_TO_POINT Task #{task.id}: No active map found."
          task.update(status: :failed, result_data: (task.result_data || {}).merge(error: "No active map for navigation."))
          return # 不向ROS发送指令
      end
    end

    instruction_to_ros = {
      command_type: command_type, # "TASK_EXECUTE" or "TASK_CANCEL"
      task: {
        id: task.id,
        type: task.task_type.to_s.upcase, # E.g., "MAP_BUILD_AUTO"
        priority: task.priority,
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
    SystemLog.log(
      :task, severity, full_message,
      "RobotTaskChannel",
      { user_id: connection.current_user&.id, task_id: task&.id }
    )
  end
end
