# app/channels/robot_commands_channel.rb
class RosCommsSubscriberChannel < ApplicationCable::Channel
  def subscribed
    # 确保只有机器人客户端可以订阅这个特殊的Channel
    unless connection.robot_client
      logger.warn "[RosCommsSubscriberChannel] Unauthorized subscription by non-robot client. Rejecting."
      reject
      return
    end
    # 订阅到 "ros_comms_channel" 流，这样就能收到 RobotTaskChannel 等广播到此流的消息
    stream_from "ros_comms_channel"
    logger.info "[RosCommsSubscriberChannel] Robot client successfully subscribed and is now streaming from 'ros_comms_channel'."
  end

  def unsubscribed
    # Any cleanup needed when the robot client unsubscribes
    logger.info "[RosCommsSubscriberChannel] Robot client unsubscribed."
  end

  # 这个 Channel 主要用于接收广播，通常不需要定义很多 'action' 方法
  # 但如果 Rails 需要主动向单个 ROS 节点发送特定消息（非广播），则可以在这里定义
  # def receive(data)
  #   # 如果ROS节点通过这个channel发送了 "message" 类型的数据
  #   logger.info "[RosCommsSubscriberChannel] Received data from robot: #{data}"
  # end
end
