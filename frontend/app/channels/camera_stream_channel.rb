# app/channels/camera_stream_channel.rb
class CameraStreamChannel < ApplicationCable::Channel
  def subscribed
    if connection.robot_client
      # ROS WSRB 节点作为 robot_client 连接到此 Channel，以便发送摄像头数据
      logger.info "[CameraStreamChannel] Robot client (WSRB) connected for camera streaming."
    elsif connection.current_user
      # 普通用户订阅摄像头流
      stream_from "camera_stream_channel"
      logger.info "[CameraStreamChannel] User #{connection.current_user.id} subscribed to camera stream."
    else
      logger.warn "[CameraStreamChannel] Unauthorized subscription attempt. Rejecting."
      reject
    end
  end

  def unsubscribed
    if connection.robot_client
      logger.info "[CameraStreamChannel] Robot client (WSRB) unsubscribed from camera streaming."
    else
      logger.info "[CameraStreamChannel] User #{connection.current_user&.id} unsubscribed from camera stream."
    end
  end

  # ROS节点调用此方法发送摄像头帧数据
  def receive_camera_frame(data)
    unless connection.robot_client
      logger.warn "[CameraStreamChannel] Unauthorized receive_camera_frame call from non-robot client."
      return
    end

    payload = data['payload']
    return unless payload

    begin
      # 验证数据完整性
      required_fields = %w[frame_id timestamp format width height data_base64 data_size]
      missing_fields = required_fields.select { |field| payload[field].nil? }
      
      if missing_fields.any?
        logger.warn "[CameraStreamChannel] Missing required fields in camera frame: #{missing_fields.join(', ')}"
        return
      end

      # 可选：数据大小验证
      if payload['data_size'] > 1_048_576 # 1MB限制
        logger.warn "[CameraStreamChannel] Camera frame too large: #{payload['data_size']} bytes"
        return
      end

      # 构建广播数据
      broadcast_data = {
        type: "camera_frame",
        frame_id: payload['frame_id'],
        timestamp: payload['timestamp'],
        format: payload['format'],
        width: payload['width'],
        height: payload['height'],
        data_url: "data:image/#{payload['format']};base64,#{payload['data_base64']}",
        data_size: payload['data_size']
      }

      # 广播给所有订阅的用户
      ActionCable.server.broadcast("camera_stream_channel", broadcast_data)
      
      # 记录调试信息（限制频率）
      @last_frame_log ||= Time.current - 5.seconds
      if Time.current - @last_frame_log > 5.seconds
        logger.debug "[CameraStreamChannel] Broadcasted camera frame #{payload['frame_id']}: #{payload['width']}x#{payload['height']}, #{payload['data_size']} bytes"
        @last_frame_log = Time.current
      end

    rescue => e
      logger.error "[CameraStreamChannel] Error processing camera frame: #{e.message}"
      logger.error e.backtrace.first(3).join("\n")
    end
  end

  # 用户请求开启/关闭摄像头流
  def toggle_camera_stream(data)
    user = connection.current_user
    unless user
      logger.warn "[CameraStreamChannel] Unauthorized toggle_camera_stream call from non-user."
      return
    end

    enable = data['enable']
    unless [true, false].include?(enable)
      transmit_error("Invalid 'enable' parameter for toggle_camera_stream.")
      return
    end

    # 构建发送给ROS的控制指令
    command_payload_to_ros = {
      command_type: "TOGGLE_CAMERA_STREAM",
      payload: { enable: enable }
    }

    # 广播到ROS通信通道
    ActionCable.server.broadcast("ros_comms_channel", command_payload_to_ros)
    
    logger.info "[CameraStreamChannel] User #{user.id} requested camera stream #{enable ? 'ON' : 'OFF'}"
    
    # 记录系统日志
    SystemLog.log(
      :robot, :info,
      "Camera stream #{enable ? 'enabled' : 'disabled'} by user #{user.id}",
      "CameraStreamChannel#toggle_camera_stream",
      { user_id: user.id, camera_enabled: enable }
    )

    # 向调用者确认
    transmit({ 
      status: "success", 
      message: "Camera stream #{enable ? 'enable' : 'disable'} request sent to robot.",
      enabled: enable
    })
  end

  # 用户请求摄像头参数调整
  def set_camera_params(data)
    user = connection.current_user
    unless user&.admin? # 假设只有管理员可以调整摄像头参数
      transmit_error("Unauthorized: Only admins can adjust camera parameters.")
      return
    end

    params = data['params'] || {}
    
    # 验证参数
    valid_params = {}
    if params['fps']
      fps = params['fps'].to_f
      if fps.between?(1.0, 15.0)
        valid_params['fps'] = fps
      else
        transmit_error("FPS must be between 1.0 and 15.0")
        return
      end
    end

    if params['quality']
      quality = params['quality'].to_i
      if quality.between?(10, 100)
        valid_params['quality'] = quality
      else
        transmit_error("Quality must be between 10 and 100")
        return
      end
    end

    return transmit_error("No valid parameters provided") if valid_params.empty?

    # 构建发送给ROS的控制指令
    command_payload_to_ros = {
      command_type: "SET_CAMERA_PARAMS",
      payload: { params: valid_params }
    }

    # 广播到ROS通信通道
    ActionCable.server.broadcast("ros_comms_channel", command_payload_to_ros)
    
    logger.info "[CameraStreamChannel] User #{user.id} adjusted camera params: #{valid_params}"
    
    # 记录系统日志
    SystemLog.log(
      :robot, :info,
      "Camera parameters adjusted by user #{user.id}: #{valid_params}",
      "CameraStreamChannel#set_camera_params",
      { user_id: user.id, camera_params: valid_params }
    )

    # 向调用者确认
    transmit({ 
      status: "success", 
      message: "Camera parameter adjustment request sent to robot.",
      params: valid_params
    })
  end

  private

  def transmit_error(message)
    logger.warn "[CameraStreamChannel] Transmitting error to client: #{message}"
    transmit({ status: "error", error: message })
  end
end 