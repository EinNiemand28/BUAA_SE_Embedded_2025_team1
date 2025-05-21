# app/controllers/robots_controller.rb
class RobotsController < ApplicationController
  # 确保用户已登录才能访问，除非是特定API端点
  before_action :authenticate, except: [ :update_status, :simulate_status ]
  # 某些操作需要管理员权限
  before_action :require_admin!, only: [ :reset_status, :emergency_stop_via_http, :resume_via_http, :set_active_map_via_http, :start_work_via_http ] # 后缀via_http区分于WebSocket操作
  # 对于外部API调用，禁用CSRF保护
  skip_before_action :verify_authenticity_token, only: [ :update_status, :simulate_status ]


  def index
    @robot_status = RobotStatus.current
    @recent_tasks = Task.order(created_at: :desc).limit(5)
    # 确保翻译正常
    # render plain: t('robots.status.idle') # 测试翻译
  end

  # 机器人控制面板页面 (包含建图UI)
  def control
    @robot_status = RobotStatus.current
    @maps = Map.all.order(created_at: :desc) # 用于地图选择下拉框
    @active_map = @robot_status.active_map

    # 如果当前机器人正在执行建图任务，则加载该任务以便前端显示其状态
    if @robot_status.status == :mapping && @robot_status.current_task&.type_map_build_auto?
      @current_mapping_task_for_view = @robot_status.current_task
    end
    # 你也可以传递一个空的Task对象给表单，如果需要
    @new_mapping_task_form_object = Task.new(task_type: :map_build_auto)
  end

  # 返回机器人当前状态 (JSON格式，用于前端轮询或API)
  def status
    @robot_status = RobotStatus.current
    respond_to do |format|
      format.html # status.html.erb (如果需要)
      format.json { render json: @robot_status.as_json(include: [ :active_map, :current_task ]) }
    end
  end

  # === 通过 HTTP POST 发起的机器人控制指令 (通常这些会通过WebSocket的RobotControlChannel) ===
  # 为了保持接口，但建议主要使用WebSocket。这些方法可以作为后备或特定场景使用。

  # HTTP POST 触发紧急停止 (示例，通常由WebSocket处理)
  def emergency_stop_via_http
    RobotStatus.current.emergency_stop! # 直接调用模型方法更新状态
    # 通过 RobotControlChannel 广播给ROS (如果ROS也订阅它，或者有其他机制)
    ActionCable.server.broadcast("ros_comms_channel", {
      command_type: "EMERGENCY_STOP", payload: { triggered_by: "http_admin_#{Current.user.id}" }
    })
    SystemLog.log(:robot, :critical, "紧急停止通过HTTP被用户 #{Current.user.id} 触发。", "RobotsController#emergency_stop_via_http", user_id: Current.user.id)
    redirect_to control_robots_path, notice: t("robots.notices.emergency_stopped_http")
  end

  # HTTP POST 触发从急停恢复 (示例)
  def resume_via_http
    if RobotStatus.current.resume_from_emergency_stop!
      ActionCable.server.broadcast("ros_comms_channel", {
        command_type: "RESUME", payload: { triggered_by: "http_admin_#{Current.user.id}" } # 假设ROS有RESUME指令
      })
      SystemLog.log(:robot, :warning, "机器人操作通过HTTP被用户 #{Current.user.id} 恢复。", "RobotsController#resume_via_http", user_id: Current.user.id)
      redirect_to control_robots_path, notice: t("robots.notices.resumed_http")
    else
      redirect_to control_robots_path, alert: t("robots.notices.cannot_resume_http", status: RobotStatus.current.status)
    end
  end

  # HTTP POST 触发设置活动地图 (改为创建 LOAD_MAP 任务)
  def set_active_map_via_http
    map_id = params[:map_id]
    map_to_activate = Map.find_by(id: map_id)

    unless map_to_activate
      redirect_to control_robots_path, alert: t("robots.notices.map_not_found")
      return
    end
    unless map_to_activate.map_config.attached? && map_to_activate.map_image.attached?
      redirect_to control_robots_path, alert: "地图 '#{map_to_activate.name}' 缺少配置文件或图像文件，无法激活。"
      return
    end

    create_load_map_task_and_broadcast(map_to_activate, Current.user, "RobotsController#set_active_map_via_http")
    redirect_to control_robots_path, notice: t("robots.notices.load_map_task_created", name: map_to_activate.name)
  end

  # HTTP POST 触发开始工作 (改为创建 LOAD_MAP 任务，如果需要加载地图)
  def start_work_via_http
    robot_s = RobotStatus.current
    active_map = robot_s.active_map

    unless active_map.present?
      redirect_to control_robots_path, alert: t("robots.notices.no_active_map")
      return
    end
    unless active_map.map_config.attached? && active_map.map_image.attached?
      redirect_to control_robots_path, alert: "活动地图 '#{active_map.name}' 缺少配置文件或图像文件。"
      return
    end

    # "开始工作" 通常意味着机器人需要加载其活动地图并进入空闲状态
    # 我们创建一个 LOAD_MAP 任务
    if robot_s.can_process_task?(:load_map) # 检查是否可以处理加载地图任务
      create_load_map_task_and_broadcast(active_map, Current.user, "RobotsController#start_work_via_http")
      # 真正的机器人状态（如idle）应该在LOAD_MAP任务成功完成后，由ROS通过RobotFeedbackChannel反馈更新
      redirect_to control_robots_path, notice: "机器人开始工作指令已发送 (加载地图: #{active_map.name})."
    else
      redirect_to control_robots_path, alert: t("robots.notices.cannot_start_work", status: t("robots.status.#{robot_s.status}"))
    end
  end

  # HTTP POST 重置机器人状态 (管理员操作，谨慎使用)
  def reset_status
    RobotStatus.current.update(
      status: :offline, # 或者 :idle，取决于重置的含义
      current_task: nil,
      error_message: "状态被管理员 #{Current.user.id} 手动重置。",
      is_emergency_stopped: false,
      is_mapping: false,
      is_navigating: false
    )
    # 可选：也向ROS发送一个通用重置指令
    ActionCable.server.broadcast("ros_comms_channel", {
      command_type: "SYSTEM_COMMAND", payload: { command: "reset_robot_state" } # 假设ROS有此指令
    })
    SystemLog.log(:robot, :critical, "机器人状态被管理员 #{Current.user.id} 手动重置。", "RobotsController#reset_status", user_id: Current.user.id)
    redirect_to control_robots_path, notice: t("robots.notices.status_reset_http")
  end


  # === 外部API接口 (需要API Key认证) ===

  # 接收来自机器人的状态更新 (例如，如果机器人不能使用WebSocket)
  # POST /robots/update_status
  # Headers: X-Robot-API-Key: YOUR_API_KEY
  # Body (JSON): { "status": "idle", "battery_level": 88.5, "pose": {"x":1.0, "y":2.5, "theta":0.5}, ... }
  def update_status
    authenticate_robot_api_key! do
      payload = params.permit!.to_h.deep_symbolize_keys # 允许所有参数并符号化
      # 这里的 payload 结构应该与 RobotFeedbackChannel#update_robot_state 期望的类似
      if RobotStatus.current.update_status_from_ros(payload)
        head :ok
      else
        render json: { errors: RobotStatus.current.errors.full_messages }, status: :unprocessable_entity
      end
    end
  end

  # 模拟状态更新 (仅用于开发和测试)
  def simulate_status
    unless Rails.env.development? || Rails.env.test?
      render json: { error: "仅在开发或测试环境可用" }, status: :forbidden
      return
    end

    simulated_payload = {
      pose: { x: rand(-10.0..10.0).round(2), y: rand(-10.0..10.0).round(2), theta: rand(0..6.28).round(2) },
      velocity: { linear: rand(0..0.5).round(2), angular: rand(-0.5..0.5).round(2) },
      battery_level: rand(10.0..100.0).round(1),
      overall_status: params[:overall_status] || RobotStatus.statuses.keys.sample, # 随机状态或指定
      error_message: params[:error_message],
      is_emergency_stopped: params[:is_emergency_stopped] == "true"
    }.compact

    # 通过 RobotFeedbackChannel 广播，就像ROS发来的一样
    # 但这里我们直接操作 RobotStatus 并让其回调广播，或直接广播到通用流
    RobotStatus.current.update_status_from_ros(simulated_payload.except(:overall_status)) # overall_status 单独处理
    if simulated_payload[:overall_status] && !RobotStatus.current.is_busy_with_task_type_specific_status?
        RobotStatus.current.update(status: simulated_payload[:overall_status])
    end


    # 或者直接广播一个通用状态给前端
    # ActionCable.server.broadcast("robot_general_updates_channel", {
    #   type: "robot_state_update", payload: simulated_payload
    # })
    render json: { message: "模拟状态更新已处理。", simulated_payload: simulated_payload }
  end


  private

  def require_admin!
    # 假设你使用的是 Current.user 并且 User 模型有 admin? 方法
    unless Current.user&.admin?
      redirect_to root_path, alert: t("common.not_authorized")
    end
  end

  # 外部API Key认证辅助方法
  def authenticate_robot_api_key!
    api_key_provided = request.headers["X-Robot-API-Key"]
    expected_api_key = ENV["ROBOT_API_KEY"]

    if api_key_provided.present? && expected_api_key.present? &&
       ActiveSupport::SecurityUtils.secure_compare(api_key_provided, expected_api_key)
      yield # 执行块内代码
    else
      render json: { error: "未授权的API访问" }, status: :unauthorized
    end
  end

  # 创建加载地图任务并广播的辅助方法
  def create_load_map_task_and_broadcast(map, user, source_description)
    task = Task.new(
      user: user,
      task_type: :load_map,
      status: :pending,
      map: map
    )
    task.store_parameters({ map_id: map.id, map_name: map.name })

    if task.save
      # 构建指令并广播给ROS
      config_url = map.map_config_access_url
      image_url = map.map_image_access_url
      unless config_url && image_url
          task.update(status: :failed, result_data: { error: "地图文件URL无法生成。" })
          SystemLog.log(:error, :error, "为地图 #{map.id} 生成地图文件URL失败。", source_description, task_id: task.id)
          # 这里应该通知调用者创建任务失败，而不是直接重定向后才显示
          # raise StandardError, "地图文件URL无法生成。" # 或者返回false
          return false # 表示失败
      end

      instruction_payload = {
        id: task.id, type: task.task_type.to_s.upcase,
        parameters: task.fetch_parameters.merge(
          map_config_url: config_url,
          map_image_url: image_url
        )
      }
      message_to_ros = { command_type: "TASK_EXECUTE", task: instruction_payload }
      ActionCable.server.broadcast("ros_comms_channel", message_to_ros)

      SystemLog.log(:task, :info, "LOAD_MAP 任务 ##{task.id} 为地图 '#{map.name}' 已创建并发送至ROS。",
                    source_description, { user_id: user&.id, map_id: map.id, task_id: task.id })
      true # 表示成功
    else
      SystemLog.log(:error, :error, "创建 LOAD_MAP 任务失败: #{task.errors.full_messages.join(', ')}",
                    source_description, { user_id: user&.id, map_id: map.id })
      # raise StandardError, "创建 LOAD_MAP 任务失败: #{task.errors.full_messages.join(', ')}"
      false # 表示失败
    end
  end
end
