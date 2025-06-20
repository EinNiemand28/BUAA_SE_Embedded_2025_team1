# app/channels/application_cable/channel.rb
module ApplicationCable
  class Channel < ActionCable::Channel::Base
    # 可以在这里添加所有 Channel 共用的逻辑，例如统一的错误处理
    # rescue_from StandardError do |exception|
    #   logger.error "[ActionCable] Channel Error: #{exception.class} - #{exception.message}"
    #   logger.error exception.backtrace.first(5).join("\n")
    #   # transmit({ error: "An unexpected error occurred in the channel." }) # 可选：通知客户端
    # end
  end
end
