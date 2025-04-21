module ApplicationCable
  class Connection < ActionCable::Connection::Base
    identified_by :current_user, :robot_client # 恢复标识

    def connect
      # 恢复原始连接逻辑
      api_key_provided = request.params[:api_key]
      expected_api_key = ENV['ROBOT_API_KEY']
      
      logger.debug "Attempting WebSocket connection..."
      logger.debug "API Key Provided: #{api_key_provided.present? ? 'Yes' : 'No'} ([FILTERED])"
      logger.debug "Expected API Key Loaded: #{expected_api_key.present? ? 'Yes' : 'No'}"

      if api_key_provided.present? && api_key_provided == expected_api_key
        logger.info "API Key matches. Identifying as robot client."
        self.robot_client = true
        self.current_user = nil # 确保机器人连接时用户为nil
      else
        logger.info "No valid API Key found or key mismatch. Attempting to identify as user."
        self.robot_client = false
        self.current_user = find_verified_user # 使用包含Warden和Cookie逻辑的方法
        logger.info "User found via find_verified_user: #{self.current_user.present? ? self.current_user.id : 'None'}"
      end

      # 最终授权检查
      if self.robot_client || self.current_user
        logger.info "Connection authorized for: #{self.robot_client ? 'Robot' : "User #{self.current_user&.id}"}"
        # ActionCable 隐式注册连接
      else
        logger.warn "Rejecting unauthorized connection. Robot flag=#{self.robot_client}, User found=#{self.current_user.present?}"
        reject_unauthorized_connection
      end
    end

    # 处理 Action Cable 内部的 ping 消息
    def send_ping
      # 阻止 Rails 记录内部 ping 为 "unrecognized command"
      # Action Cable 会自动处理 pong 响应
      # logger.debug "[Connection] Sending internal ping"
      super
    end

    # 如果需要处理客户端发送的 ping (type: "ping")
    # 注意：标准 WebSocket ping 是控制帧，通常由服务器/库自动处理
    # Action Cable 的 {type: "ping"} 是其自身协议的一部分
    # def receive(websocket_message)
    #   message = ActiveSupport::JSON.decode(websocket_message)
    #   if message['type'] == 'ping'
    #     # logger.debug "[Connection] Received Action Cable ping message. Ignoring."
    #     # 不需要手动发送 pong，Action Cable 会处理
    #     return 
    #   end
    #   # 如果不是 ping，则交给正常的命令分发
    #   super
    # end

    private
      def find_verified_user
        # 1. 尝试使用 URL token
        token = request.params[:user_token]
        logger.debug "find_verified_user: Attempting token verification. Token present: #{token.present?}"
        if token
          begin
            # 使用 find_signed 是推荐的方式，它处理 nil 情况
            user = User.find_signed(token, purpose: :action_cable_connection)
            if user
              logger.info "Authenticated user via signed token: #{user.id}"
              return user
            else
              # find_signed 返回 nil 如果 token 无效或过期
              logger.warn "User token verification failed (invalid, expired, or wrong purpose). Token: #{token}"
            end
          rescue => e
            logger.error "Error during User.find_signed with token: #{e.message} Backtrace: #{e.backtrace.first(5).join(" | ")}"
          end
        else
          logger.debug "find_verified_user: No user_token found in request params."
        end

        # 2. 备选：尝试 Warden / Devise session (可能仍然不可靠)
        if warden_proxy = request.env['warden']
          user = warden_proxy.user(:user) # :user 是默认 scope
          if user 
             logger.debug "Found user via env['warden'] (fallback 1): #{user.id}"
             return user 
          end
        end

        # 3. 最后备选：尝试 cookie (通常在 WebSocket 握手时最不可靠)
        user = nil
        begin
          if (verified_user_id = cookies.encrypted[:user_id])
            user = User.find_by(id: verified_user_id)
            logger.debug "Found user ID in cookies (fallback 2): #{verified_user_id}" if user
          else 
             # logger.debug "No user ID found in cookies (fallback 2)." #减少冗余日志
          end
        rescue => e
           logger.error "Error finding verified user via cookies (fallback 2): #{e.message}"
        end
        
        logger.warn "Unable to find verified user via token, warden, or cookies." unless user
        user # 返回找到的用户或 nil
      end
  end
end
