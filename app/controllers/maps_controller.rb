# app/controllers/maps_controller.rb
class MapsController < ApplicationController
  before_action :authenticate
  before_action :require_admin!, except: [ :index, :show ] # 大部分操作需要管理员权限
  before_action :set_map, only: [ :show, :edit, :update, :destroy, :activate_via_http ]

  def index
    @maps = Map.all.includes(:created_by_user, :thumbnail_attachment).order(created_at: :desc)
    @active_map = Map.active_map # 获取当前活动地图
    # @maps = @maps.page(params[:page]).per(10) # Kaminari示例
  end

  def show
    # @map 由 set_map 设置
    # 获取与此地图相关的任务 (例如，创建此地图的任务，或使用此地图的任务)
    # @creating_task = @map.task # 如果 Map has_one :task (创建任务)
    # @related_tasks = Task.where(map_id: @map.id).or(Task.where_assoc_exists(:child_tasks, map_id: @map.id)) # 示例
  end

  def new
    @map = Map.new
  end

  def create
    @map = Map.new(map_params)
    @map.created_by_user = Current.user # 关联当前用户

    if @map.save
      SystemLog.log(:map_event, :info, "地图 ##{@map.id} '#{@map.name}' 由用户 #{Current.user.id} 创建。",
                    "MapsController#create", { user_id: Current.user.id, map_id: @map.id })
      redirect_to @map, notice: t("maps.notices.created_successfully")
    else
      render :new, status: :unprocessable_entity
    end
  end

  def edit
    # @map 由 set_map 设置
  end

  def update
    if @map.update(map_params)
      SystemLog.log(:map_event, :info, "地图 ##{@map.id} '#{@map.name}' 由用户 #{Current.user.id} 更新。",
                    "MapsController#update", { user_id: Current.user.id, map_id: @map.id })
      redirect_to @map, notice: t("maps.notices.updated_successfully")
    else
      render :edit, status: :unprocessable_entity
    end
  end

  def destroy
    # 检查地图是否是当前活动地图
    if @map.is_active?
      redirect_to maps_path, alert: t("maps.notices.cannot_delete_active") and return
    end
    # （可选）检查是否有关联的未完成任务正在使用此地图
    # if Task.where(map_id: @map.id).where.not(status: [:completed, :failed, :cancelled]).exists?
    #   redirect_to maps_path, alert: t('maps.notices.cannot_delete_used_by_tasks') and return
    # end

    # ActiveStorage 附件会在模型销毁时自动删除 (如果配置正确)
    # @map.map_config.purge if @map.map_config.attached?
    # @map.map_image.purge if @map.map_image.attached?
    # @map.thumbnail.purge if @map.thumbnail.attached?

    @map.destroy
    SystemLog.log(:map_event, :warning, "地图 ##{@map.id} '#{@map.name}' 被用户 #{Current.user.id} 删除。",
                  "MapsController#destroy", { user_id: Current.user.id, map_id: @map.id }) # map_id 在删除后不可用，所以记录ID
    redirect_to maps_path, notice: t("maps.notices.deleted_successfully")
  end

  # HTTP POST 触发激活地图 (改为创建 LOAD_MAP 任务)
  # 注意：方法名改为 activate_via_http 以区分可能的WebSocket操作，路由也应对应
  def activate_via_http
    # @map 由 set_map 设置
    unless @map.map_config.attached? && @map.map_image.attached?
      redirect_to maps_path, alert: "地图 '#{@map.name}' 缺少配置文件或图像文件，无法激活。"
      return
    end

    # 创建 LOAD_MAP 任务的逻辑与 RobotsController 中的类似
    task = Task.new(
      user: Current.user,
      task_type: :load_map,
      status: :pending,
      map: @map # 关联要激活的地图
    )
    task.store_parameters({ map_id: @map.id, map_name: @map.name })

    if task.save
      # 构建指令并广播给ROS (与TasksController和RobotsController中的逻辑一致)
      config_url = @map.map_config_access_url
      image_url = @map.map_image_access_url
      unless config_url && image_url
          task.update(status: :failed, result_data: { error: "地图文件URL无法从MapsController生成。" })
          redirect_to maps_path, alert: "生成地图文件URL失败，无法创建加载任务。" and return
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

      # 注意：数据库层面的 is_active 标志应该在 :load_map 任务成功完成后，
      # 由 RobotFeedbackChannel 接收到ROS的确认后，再调用 @map.activate_in_db! 来设置。
      # 这里只是创建了任务。

      SystemLog.log(:map_event, :info, "请求加载地图 ##{@map.id} '#{@map.name}' 的任务 ##{task.id} 已创建。",
                    "MapsController#activate_via_http", { user_id: Current.user.id, map_id: @map.id, task_id: task.id })
      redirect_to maps_path, notice: t("maps.notices.activation_task_created_successfully", map_name: @map.name)
    else
      redirect_to maps_path, alert: t("maps.notices.activation_task_failed", errors: task.errors.full_messages.join(", "))
    end
  end


  private

  def set_map
    @map = Map.find(params[:id])
  rescue ActiveRecord::RecordNotFound
    redirect_to maps_path, alert: t("maps.notices.not_found")
  end

  def map_params
    # 允许的参数，特别是对于 ActiveStorage 附件
    # 对于附件，通常表单会直接提交文件对象
    params.require(:map).permit(:name, :description, :map_config, :map_image, :thumbnail, :is_active)
    # :map_data_url 字段可能不再需要，如果完全依赖 ActiveStorage
  end

  def require_admin!
    unless Current.user&.admin?
      redirect_to root_path, alert: t("common.not_authorized")
    end
  end
end
