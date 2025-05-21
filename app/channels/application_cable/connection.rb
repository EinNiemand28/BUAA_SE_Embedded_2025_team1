# app/channels/application_cable/connection.rb
module ApplicationCable
  class Connection < ActionCable::Connection::Base
    identified_by :current_user, :robot_client

    def connect
      self.robot_client = identify_robot_client
      if self.robot_client
        logger.info "[ActionCable] Robot client connected via API Key."
        self.current_user = nil # 机器人连接时不应有关联用户
      else
        self.current_user = find_verified_user_for_cable
        if self.current_user
          logger.info "[ActionCable] User connected: #{current_user.id} (#{current_user.username})."
        else
          logger.warn "[ActionCable] Anonymous or unverified user connection attempt."
        end
      end

      # 最终授权检查：允许已识别的机器人或已登录的用户连接
      unless self.robot_client || self.current_user
        logger.warn "[ActionCable] Rejecting unauthorized connection. No robot client and no verified user."
        reject_unauthorized_connection
      end
    end

    private

    def identify_robot_client
      api_key_provided = request.headers["X-Robot-API-Key"]
      expected_api_key = ENV["ROBOT_API_KEY"]

      if api_key_provided.present? && expected_api_key.present? &&
         ActiveSupport::SecurityUtils.secure_compare(api_key_provided, expected_api_key)
        return true
      end
      false
    end

    def find_verified_user_for_cable
      # 优先使用 URL token (更适合 WebSocket 这种无 cookie 场景)
      token = request.params[:user_token]
      if token.present?
        user = User.find_by_token_for(:action_cable_connection, token)
        if user
          logger.info "[ActionCable] Authenticated user via find_by_token_for(:action_cable_connection): #{user.id}"
          return user
        else
          logger.warn "[ActionCable] User token verification failed (invalid, expired, or wrong purpose)."
        end
      end

      # 备选：尝试 Warden (如果 WebSocket 握手时 Warden 信息可用)
      # 注意：这在生产环境中可能不可靠，因为WebSocket连接可能不经过Warden的完整HTTP请求周期
      # if defined?(env["warden"]) && (warden_user = env["warden"].user(:user))
      #   logger.info "[ActionCable] Authenticated user via Warden: #{warden_user.id}"
      #   return warden_user
      # end

      # 最后备选：尝试从 cookies 中恢复用户 (最不可靠，但可以作为后备)
      # Rails 7+ 默认 ActionCable 不会传递 cookies
      # if (verified_user_id = cookies.encrypted[:user_id])
      #   user = User.find_by(id: verified_user_id)
      #   logger.info "[ActionCable] Authenticated user via encrypted cookie: #{user.id}" if user
      #   return user
      # end

      nil # 没有找到已验证的用户
    end
  end
end
