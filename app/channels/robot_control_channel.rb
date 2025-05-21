# app/channels/robot_control_channel.rb
class RobotControlChannel < ApplicationCable::Channel
  def subscribed
    unless connection.current_user
      logger.warn "[RobotControlChannel] Unauthorized subscription: No user. Rejecting."
      reject
      return
    end
    logger.info "[RobotControlChannel] User #{connection.current_user.id} subscribed."
    # 不需要 stream_from，因为这个 Channel 主要用于接收前端 perform 的 action
  end

  def unsubscribed
    logger.info "[RobotControlChannel] User #{connection.current_user&.id} unsubscribed."
  end

  # --- 移动控制 ---
  def move(data)
    user = connection.current_user
    return transmit_error("User not authenticated for MOVE.") unless user

    # 注意：实际是否能移动由 TaskManager 根据其当前状态 (是否manual_control) 决定
    # Rails 端只负责转发指令
    direction = data["direction"]
    speed = data["speed"].to_f

    unless [ "forward", "backward", "left", "right", "stop" ].include?(direction)
      return transmit_error("Invalid move direction: #{direction}")
    end
    speed = speed.clamp(-1.0, 1.0)

    command_payload_to_ros = {
      command_type: "MOVE", # TM 会识别这个 command_type
      payload: { direction: direction, speed: speed }
    }
    broadcast_command_to_ros(command_payload_to_ros, "Move command: #{direction}, speed #{speed}")
    # 可选：向调用者前端发送一个快速确认
    # transmit({ status: "info", message: "Move command sent to robot bridge."})
  end

  def stop_motion(_data)
    user = connection.current_user
    return transmit_error("User not authenticated for STOP_MOTION.") unless user

    command_payload_to_ros = {
      command_type: "MOVE", # TM会将 direction: "stop" 理解为停止
      payload: { direction: "stop", speed: 0.0 }
    }
    broadcast_command_to_ros(command_payload_to_ros, "Stop motion command")
  end

  # --- 紧急停止与恢复 ---
  def emergency_stop(_data)
    user = connection.current_user
    # 权限检查：例如，只有管理员能触发急停
    unless user && user.admin?
      return transmit_error("Unauthorized: Only admins can trigger emergency stop.")
    end

    command_payload_to_ros = { command_type: "EMERGENCY_STOP", payload: {} }
    broadcast_command_to_ros(command_payload_to_ros, "EMERGENCY STOP command initiated", :critical)

    # Rails 端也应立即尝试更新 RobotStatus (TM的反馈会最终确认)
    # 这有助于UI快速反应，即使ROS反馈有延迟
    RobotStatus.current.emergency_stop! # 调用模型方法
    transmit({ status: "success", message: "Emergency stop signal sent and local status updated." })
  end

  def resume_operation(_data) # 从急停状态恢复到 Idle
    user = connection.current_user
    unless user && user.admin? # 假设也需要管理员权限来恢复
      return transmit_error("Unauthorized: Only admins can resume operation from E-STOP.")
    end

    # 检查Rails端 RobotStatus 是否处于急停（可选，主要由TM判断）
    # unless RobotStatus.current.status_emergency_stopped?
    #   return transmit_error("Robot is not currently in emergency stop state (according to Rails).")
    # end

    command_payload_to_ros = { command_type: "RESUME_OPERATION", payload: {} }
    broadcast_command_to_ros(command_payload_to_ros, "Resume operation command initiated", :warning)
    # Rails 端的 RobotStatus 更新将依赖于TM的反馈
    transmit({ status: "success", message: "Resume operation signal sent." })
  end

  # --- 手动控制模式切换 ---
  def enable_manual_control(_data) # 从 急停 -> 手动
    user = connection.current_user
    unless user && user.admin? # 假设需要管理员权限
      return transmit_error("Unauthorized: Only admins can enable manual control mode.")
    end

    # 检查Rails端 RobotStatus （可选，主要由TM判断能否切换）
    # unless RobotStatus.current.status_emergency_stopped?
    #   return transmit_error("Cannot enable manual control: Robot must be in emergency stop state first (according to Rails).")
    # end

    command_payload_to_ros = { command_type: "ENABLE_MANUAL_CONTROL", payload: {} }
    broadcast_command_to_ros(command_payload_to_ros, "Enable manual control command initiated")
    transmit({ status: "success", message: "Enable manual control signal sent." })
  end

  def disable_manual_control(_data) # 从 手动 -> 空闲
    user = connection.current_user
    unless user && user.admin? # 假设需要管理员权限
      return transmit_error("Unauthorized: Only admins can disable manual control mode.")
    end

    # 检查Rails端 RobotStatus （可选）
    # unless RobotStatus.current.status_manual_control?
    #   return transmit_error("Robot is not currently in manual control mode (according to Rails).")
    # end

    command_payload_to_ros = { command_type: "DISABLE_MANUAL_CONTROL", payload: {} }
    broadcast_command_to_ros(command_payload_to_ros, "Disable manual control command initiated (return to idle)")
    transmit({ status: "success", message: "Disable manual control signal sent." })
  end


  # toggle_camera_stream 等其他即时控制可以保持不变
  def toggle_camera_stream(data)
    user = connection.current_user
    return transmit_error("User not authenticated for TOGGLE_CAMERA.") unless user
    enable = data["enable"]
    unless [ true, false ].include?(enable)
      return transmit_error("Invalid 'enable' for toggle_camera_stream.")
    end
    command_payload_to_ros = { command_type: "TOGGLE_CAMERA", payload: { enable: enable } }
    broadcast_command_to_ros(command_payload_to_ros, "Toggle camera: #{enable}")
  end


  private

  def broadcast_command_to_ros(message_to_ros_node, log_message_prefix, severity = :info)
    # 所有来自此Channel的指令都通过 "ros_comms_channel" 流广播出去
    # web_robot_bridge_node.py 会在其订阅的 RosCommsSubscriberChannel 中接收这些
    ActionCable.server.broadcast("ros_comms_channel", message_to_ros_node)

    log_details = "Command: #{message_to_ros_node[:command_type]}"
    log_details += ", Payload: #{message_to_ros_node[:payload].to_json}" if message_to_ros_node[:payload].present?

    logger.info "[RobotControlChannel] Broadcasted to 'ros_comms_channel': #{log_details}"
    SystemLog.log(:robot, severity,
                  "#{log_message_prefix} (User: #{connection.current_user&.id}) -> ROS. #{log_details}",
                  "RobotControlChannel##{message_to_ros_node[:command_type].downcase}",
                  { user_id: connection.current_user&.id })
  end

  def transmit_error(messages)
    errors_array = Array(messages)
    logger.warn "[RobotControlChannel] Transmitting error to client: #{errors_array.join(', ')}"
    transmit({ status: "error", errors: errors_array }) # 向调用者JS回传错误
  end
end
