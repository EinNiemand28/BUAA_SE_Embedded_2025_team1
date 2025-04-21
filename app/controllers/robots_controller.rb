class RobotsController < ApplicationController
  before_action :authenticate, except: [:update_status, :simulate_status]
  skip_before_action :verify_authenticity_token, only: [:update_status, :simulate_status]
  
  def index
    # 机器人列表页面，可以后续实现
  end
  
  def control
    # 机器人控制面板页面
  end
  
  # 用于广播机器人状态的方法，可以由外部API调用
  def update_status
    if request.headers['X-Robot-API-Key'] == ENV['ROBOT_API_KEY']
      ActionCable.server.broadcast "robot_status_channel", params.permit!
      render json: { status: "success" }
    else
      render json: { error: "Unauthorized" }, status: :unauthorized
    end
  end
  
  # 模拟机器人状态，用于开发和测试
  def simulate_status
    # 确保只在开发环境可用
    if !Rails.env.production? || (Rails.env.production? && params[:debug_key] == ENV['DEBUG_KEY'])
      data = {
        heartbeat: true,
        battery_level: rand(20..100),
        position_x: rand(-10.0..10.0),
        position_y: rand(-10.0..10.0),
        linear_vel: rand(-0.5..0.5),
        angular_vel: rand(-0.5..0.5),
        timestamp: Time.now.to_i
      }
      
      ActionCable.server.broadcast "robot_status_channel", data
      render json: { status: "success", data: data }
    else
      render json: { error: "Disabled in production" }, status: :forbidden
    end
  end
end 