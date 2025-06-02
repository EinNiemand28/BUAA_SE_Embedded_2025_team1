# app/controllers/maps_controller.rb
class MapsController < ApplicationController
  before_action :authenticate
  before_action :require_admin!, except: [ :show ] # 大部分操作需要管理员权限
  before_action :set_map, except: [ :index, :new, :create ]

  def index
    @maps = Map.all.includes(:created_by_user, :map_image_attachment).order(created_at: :desc)
    @active_map = Map.active_map
    @maps = @maps.page(params[:page]).per(6) # Kaminari示例
  end

  def show
  end

  def new
    @map = Map.new
  end

  def create
    @map = Map.new(map_params)

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
    # 处理激活地图的请求
    if params[:activate] == "true"
      if @map.activate_in_db!
        SystemLog.log(:map_event, :info,
          "地图 ##{@map.id} '#{@map.name}' 由用户 #{Current.user.id} 激活。",
          "MapsController#update", { user_id: Current.user.id, map_id: @map.id })
        redirect_to @map, notice: t("maps.notices.activated")
      else
        redirect_to @map, alert: "激活地图失败：#{@map.errors.full_messages.join(', ')}"
      end
      return
    end

    if @map.update(map_params_for_update)
      SystemLog.log(:map_event, :info,
        "地图 ##{@map.id} '#{@map.name}' 由用户 #{Current.user.id} 更新。",
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
  private

  def set_map
    @map = Map.find(params[:id])
  rescue ActiveRecord::RecordNotFound
    redirect_to maps_path, alert: t("maps.notices.not_found")
  end

  def map_params
    params.require(:map).permit(:name, :map_data_url, :created_by_user_id, :description, :map_image)
  end

  def map_params_for_update
    params.require(:map).permit(:description)
  end

  def require_admin!
    unless Current.user&.admin?
      redirect_to root_path, alert: t("common.not_authorized")
    end
  end
end
