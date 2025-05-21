# app/helpers/application_helper.rb
module ApplicationHelper
  # 封装 heroicon gem 的辅助方法，以防初始化器不起作用
  def heroicon(name, options = {})
    # 首先尝试使用 gem 提供的辅助方法
    if respond_to?(:heroicon_tag)
      heroicon_tag(name, options)
    else
      # 如果 gem 提供的方法不可用，提供一个简单的回退
      options[:class] ||= ""
      icon_class = "heroicon #{options[:class]}"
      content_tag(:i, nil, class: icon_class, title: name, data: { icon: name })
    end
  end

  def status_badge_class(status_string)
    case status_string.to_s # Ensure it's a string for case comparison
    when "pending", "cancelling"
      "bg-yellow-100 text-yellow-800"
    when "processing"
      "bg-blue-100 text-blue-800"
    when "completed"
      "bg-green-100 text-green-800"
    when "failed", "error" # Assuming your RobotStatus might also have 'error'
      "bg-red-100 text-red-800"
    when "cancelled"
      "bg-gray-100 text-gray-700" # Or another color for cancelled
    when "offline", "emergency_stopped" # RobotStatus specific
      "bg-red-100 text-red-700"
    when "idle" # RobotStatus specific
      "bg-green-100 text-green-700"
    when "mapping", "navigating", "fetching_book", "returning_book", "scanning", "paused" # RobotStatus specific
      "bg-indigo-100 text-indigo-800" # Example for busy states
    else
      "bg-gray-100 text-gray-800" # Default
    end
  end
end
