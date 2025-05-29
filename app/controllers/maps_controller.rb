# app/controllers/maps_controller.rb
class MapsController < ApplicationController
  before_action :authenticate
  before_action :require_admin!, except: [ :index, :show ] # 大部分操作需要管理员权限
  before_action :set_map, only: [ :show, :edit, :update, :destroy, :activate_via_http ]

  def index
    @maps = Map.all.includes(:created_by_user, :thumbnail_attachment).order(created_at: :desc)
    @active_map = Map.active_map
    # @maps = @maps.page(params[:page]).per(10) # Kaminari示例
  end

  def show
  end

  def new
    @map = Map.new
  end

  def create
    @map = Map.new(map_params)
    @map.created_by_user = Current.user # 关联当前用户

    if @map.save
      SystemLog.log(:map_event, :info,
        "地图 ##{@map.id} '#{@map.name}' 由用户 #{Current.user.id} 创建。",
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
    # 现在不允许直接更新
  end

  def destroy
    # 检查地图是否是当前活动地图
    if @map.is_active?
      redirect_to maps_path, alert: t("maps.notices.cannot_delete_active") and return
    end
    # 检查是否有关联的未完成任务正在使用此地图
    if Task.where(map_id: @map.id).where.not(status: [ :completed, :failed, :cancelled ]).exists?
      redirect_to maps_path, alert: t("maps.notices.cannot_delete_used_by_tasks") and return
    end

    SystemLog.log(:map_event, :warning,
      "地图 ##{@map.id} '#{@map.map_data_url}' 被用户 #{Current.user.id} 删除。",
      "MapsController#destroy", { user_id: Current.user.id, map_id: @map.id })
    @map.destroy
    redirect_to maps_path, notice: t("maps.notices.deleted_successfully")
  end

  # HTTP POST 触发激活地图 (创建 LOAD_MAP 任务)
  # 注意：方法名改为 activate_via_http 以区分可能的WebSocket操作，路由也应对应
  def activate_via_http
    # 暂时不实现
  end

  private

  def set_map
    @map = Map.find(params[:id])
  rescue ActiveRecord::RecordNotFound
    redirect_to maps_path, alert: t("maps.notices.not_found")
  end

  def map_params
    params.require(:map).permit(:name, :map_data_url, :created_by_user_id, :description, :map_image)
  end

  def require_admin!
    unless Current.user&.admin?
      redirect_to root_path, alert: t("common.not_authorized")
    end
  end
end
