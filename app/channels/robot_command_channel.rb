class RobotCommandChannel < ApplicationCable::Channel
  def subscribed
    stream_from "robot_command_channel"
  end

  def unsubscribed
    # Any cleanup needed when channel is unsubscribed
  end

  # 仅处理特定动作，忽略其他
  # def receive(data)
  #   # 接收到Web客户端的命令并转发
  #   # ActionCable.server.broadcast "robot_command_channel", data
  #   # 上面的方法会广播所有消息，包括 ping
  #   Rails.logger.debug "[RobotCommandChannel] Received data: #{data}"
  #   # 如果需要，可以添加对特定命令类型的检查
  # end

  def move(data)
    # 处理移动命令
    message_data = { command: "move", direction: data['direction'], speed: data['speed'] }
    Rails.logger.debug "[RobotCommandChannel] Broadcasting move command: #{message_data}"
    ActionCable.server.broadcast "robot_command_channel", message_data
  end
  
  def stop(data)
    # 处理停止命令
    message_data = { command: "move", direction: "stop", speed: 0 }
    Rails.logger.debug "[RobotCommandChannel] Broadcasting stop command: #{message_data}"
    ActionCable.server.broadcast "robot_command_channel", message_data
  end
end 