# app/channels/robot_general_updates_channel.rb
class RobotGeneralUpdatesChannel < ApplicationCable::Channel
  def subscribed
    # 任何人都可以订阅这个流来接收通用的机器人状态更新
    stream_from "robot_general_updates_channel"
    logger.info "[RobotGeneralUpdatesChannel] Client (User: #{connection.current_user&.id}) subscribed."
  end

  def unsubscribed
    # Any cleanup if needed
  end
end
