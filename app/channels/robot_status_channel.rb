class RobotStatusChannel < ApplicationCable::Channel
  def subscribed
    # Log when a client successfully subscribes to this channel
    logger.info "Client subscribed to RobotStatusChannel: #{connection.current_user&.id || (connection.robot_client ? 'Robot' : 'Unknown')}"
    stream_from "robot_status_channel"
  end

  def unsubscribed
    # Any cleanup needed when channel is unsubscribed
    logger.info "Client unsubscribed from RobotStatusChannel: #{connection.current_user&.id || (connection.robot_client ? 'Robot' : 'Unknown')}"
  end

  # 定义动作以接收来自ROS节点的状态更新并广播
  def update_status(data)
    # 确保只有已识别的机器人客户端可以调用此方法
    if connection.robot_client
      payload = data['payload']
      if payload
        # 在服务器端执行广播
        ActionCable.server.broadcast "robot_status_channel", payload
        # 可以添加日志确认广播
        # logger.debug "[RobotStatusChannel] Broadcasted status update: #{payload}"
      else
        logger.warn "[RobotStatusChannel] Received update_status without payload from robot."
      end
    else
      logger.warn "[RobotStatusChannel] Unauthorized attempt to call update_status by non-robot client."
      # 可以考虑断开非机器人客户端的连接
      # connection.close
    end
  end

  # Remove the custom receive method. Action Cable should route messages
  # sent with command: "message" and the correct identifier directly to the stream.
  # def receive(data)
  #   Rails.logger.debug "[RobotStatusChannel] Received unexpected data: #{data}"
  # end
end 