class RobotsController < ApplicationController
  before_action :authenticate
  before_action :require_admin!, only: [ :control ]

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
  # 目前已不再使用

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
end
