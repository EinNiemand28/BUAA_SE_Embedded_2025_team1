# app/controllers/tasks_controller.rb
class TasksController < ApplicationController
  before_action :authenticate
  before_action :set_task, only: [ :show, :edit, :update, :destroy, :cancel, :test_update ]
  # 检查创建特定类型任务的权限
  before_action :check_task_creation_permissions, only: [ :new, :create ]

  def index
    @tasks = if Current.user.admin?
               Task.all.includes(:user, :book, :source_slot, :target_slot, :map).order(created_at: :desc)
    else
               Current.user.tasks.includes(:book, :source_slot, :target_slot, :map).order(created_at: :desc)
    end
    # 可以添加分页: @tasks = @tasks.page(params[:page]).per(20) # Kaminari示例
  end

  def show
    @logs = SystemLog.where(task_id: @task.id).order(created_at: :desc)
    # 确保前端JS可以访问到 task_id (如果TaskUpdateChannel需要)
    # 例如，在视图中: <div data-task-detail-id="<%= @task.id %>">
  end

  def new
    @task = Task.new(task_type: params[:task_type]) # 允许通过参数预设类型
    load_form_collections # 加载表单所需的下拉列表数据
  end

  def create
    @task = Current.user.tasks.build(task_params_for_create)
    # progress_details 应包含来自表单的原始 "parameters"
    # task_params_for_create 已经包含了 progress_details（如果表单中以 task[progress_details_attributes] 形式提交）
    # 或者，如果表单直接提交 parameters hash:
    if params[:task][:parameters].present? && params[:task][:parameters].is_a?(ActionController::Parameters)
        @task.store_parameters(params[:task][:parameters].permit!.to_h) # 允许所有参数并转换为Hash
    end
    @task.status ||= :pending # 确保默认状态

    # 对于特定任务类型，进行额外处理或验证
    if @task.type_load_map?
        map_id_param = @task.fetch_parameters["map_id"]
        unless map_id_param.present? && (@task.map = Map.find_by(id: map_id_param))
            @task.errors.add(:base, "加载地图任务必须指定一个有效的地图ID。")
            load_form_collections
            render :new, status: :unprocessable_entity and return
        end
    end

    # 检查机器人状态是否允许处理此任务
    robot_s = RobotStatus.current
    unless robot_s.can_process_task?(@task.task_type&.to_sym)
      flash.now[:alert] = t("tasks.notices.invalid_robot_status", status: t("robots.status.#{robot_s.status}", default: robot_s.status))
      load_form_collections
      render :new, status: :unprocessable_entity and return
    end

    if @task.save
      SystemLog.log(:task_event, :info, "任务 ##{@task.id} (#{@task.task_type}) 由用户 #{Current.user.id} 创建。",
                    "TasksController#create", { user_id: Current.user.id, task_id: @task.id })

      # 如果任务需要机器人执行，则广播指令给ROS
      if needs_robot_execution?(@task.task_type)
        broadcast_task_to_ros_from_controller("TASK_EXECUTE", @task)
      end

      # 更新机器人状态以指派任务 (如果适用)
      # robot_s.assign_task(@task) if needs_robot_execution?(@task.task_type)


      respond_to do |format|
        format.html { redirect_to @task, notice: t("tasks.notices.created_successfully") }
        format.json { render json: { status: "success", task_id: @task.id, task: @task.as_json }, status: :created }
      end
    else
      load_form_collections
      respond_to do |format|
        format.html { render :new, status: :unprocessable_entity }
        format.json { render json: { status: "error", errors: @task.errors.full_messages }, status: :unprocessable_entity }
      end
    end
  end

  def edit
    load_form_collections
    # 将 progress_details 中的 parameters 部分提取出来，方便表单填充
    @task_parameters = @task.fetch_parameters
  end

  def update
    # 不允许更改 task_type 和 user_id
    # status 的更改应主要由系统或ROS反馈驱动，用户不应直接修改为 completed/failed 等
    # 用户可能可以修改 priority, scheduled_at, 或任务参数 (progress_details)

    # 安全地提取允许用户修改的参数
    allowed_params = task_params_for_update

    # 更新 progress_details 中的 parameters 部分
    if params[:task][:parameters].present? && params[:task][:parameters].is_a?(ActionController::Parameters)
        # 只合并 parameters 部分，不覆盖整个 progress_details
        current_details = @task.progress_details.is_a?(Hash) ? @task.progress_details : {}
        new_parameters = params[:task][:parameters].permit!.to_h
        current_details["parameters"] = (current_details["parameters"] || {}).merge(new_parameters)
        allowed_params[:progress_details] = current_details
    end

    if @task.update(allowed_params)
      SystemLog.log(:task_event, :info, "任务 ##{@task.id} 由用户 #{Current.user.id} 更新。",
                    "TasksController#update", { user_id: Current.user.id, task_id: @task.id })
      redirect_to @task, notice: t("tasks.notices.updated_successfully")
    else
      load_form_collections
      @task_parameters = @task.fetch_parameters # 重新加载参数以便表单回填
      render :edit, status: :unprocessable_entity
    end
  end

  def destroy
    # 增加权限检查和任务状态检查
    unless Current.user.admin? || @task.user_id == Current.user.id
      redirect_to tasks_path, alert: t("common.not_authorized") and return
    end
    if @task.status_processing? || @task.status_cancelling?
      redirect_to @task, alert: t("tasks.notices.cannot_delete_active") and return
    end

    @task.destroy
    SystemLog.log(:task_event, :warning, "任务 ##{@task.id} (#{task.task_type}) 被用户 #{Current.user.id} 删除。",
                  "TasksController#destroy", { user_id: Current.user.id, task_id: @task.id })
    redirect_to tasks_path, notice: t("tasks.notices.deleted_successfully")
  end

  def cancel
    # 权限检查已在 set_task 中部分完成 (创建者或管理员)
    unless @task.can_be_cancelled?
      redirect_to @task, alert: t("tasks.notices.cannot_cancel_status", status: t("tasks.statuses.#{@task.status}")) and return
    end

    if @task.update(status: :cancelling) # 先标记为取消中
      SystemLog.log(:task_event, :info, "任务 ##{@task.id} 取消请求由用户 #{Current.user.id} 发起。",
                    "TasksController#cancel", { user_id: Current.user.id, task_id: @task.id })

      # 如果任务需要机器人执行，则广播取消指令给ROS
      if needs_robot_execution?(@task.task_type)
        broadcast_task_to_ros_from_controller("TASK_CANCEL", @task)
      else
        # 如果是不需要机器人执行的任务，可以直接标记为已取消
        @task.update(status: :cancelled, completed_at: Time.current)
      end
      redirect_to @task, notice: t("tasks.notices.cancel_request_sent")
    else
      redirect_to @task, alert: t("tasks.notices.cancel_failed", errors: @task.errors.full_messages.join(", "))
    end
  end

  # 测试用：手动更新任务状态 (仅管理员)
  def test_update
    unless Current.user.admin?
      redirect_to @task, alert: t("common.not_authorized") and return
    end

    new_status = params[:status].to_s.to_sym
    if Task.statuses.key?(new_status)
      update_attrs = { status: new_status }
      update_attrs[:completed_at] = Time.current if [ :completed, :failed, :cancelled ].include?(new_status) && @task.completed_at.nil?
      update_attrs[:started_at] = Time.current if new_status == :processing && @task.started_at.nil?

      @task.update(update_attrs)
      # 可以在这里模拟添加 progress_details
      redirect_to @task, notice: "测试：任务状态已更新为 #{new_status}。"
    else
      redirect_to @task, alert: "测试：无效的状态 '#{params[:status]}'"
    end
  end

  private

  def set_task
    @task = Task.find(params[:id])
  rescue ActiveRecord::RecordNotFound
    redirect_to tasks_path, alert: t("tasks.notices.not_found")
  end

  # 加载表单所需的选项集合
  def load_form_collections
    @task_type_options = Task.task_types.keys.map { |type| [ t("tasks.types.#{type}", default: type.humanize), type ] }
    # 仅管理员可以选择某些任务类型
    unless Current.user.admin?
        admin_only_types = [ :map_build_auto, :inventory_scan_and_relocate, :load_map ] # 示例
        @task_type_options.reject! { |option| admin_only_types.include?(option[1].to_sym) }
    end
    @book_options = Book.order(:title).pluck(:title, :id)
    @slot_options = Slot.joins(:bookshelf).order("bookshelves.code, slots.level, slots.row")
                       .map { |s| [ "#{s.bookshelf.code} - L#{s.level}R#{s.row}", s.id ] }
    @map_options = Map.order(:name).pluck(:name, :id)
    @user_options = User.order(:username).pluck(:username, :id) if Current.user.admin? # 仅管理员可指定用户
  end

  # 定义创建任务时允许的参数
  def task_params_for_create
    # 注意：progress_details 通常包含一个 'parameters' 哈希，这部分在action中单独处理
    # task_type, status, priority, user_id (自动设置), book_id, source_slot_id, target_slot_id, map_id, parent_task_id, scheduled_at
    # target_point_x,y,z 也应放入 parameters 哈希中，然后存入 progress_details
    base_params = params.require(:task).permit(
      :task_type, :priority, :book_id,
      :source_slot_id, :target_slot_id, :map_id,
      :parent_task_id, :scheduled_at
      # 不直接允许 :status, :progress_details, :result_data 从表单大规模赋值
    )
    # 如果表单中确实有 target_point_x 等字段，应在这里合并到 parameters 中
    # 例如，如果 parameters 是一个单独的表单字段 (如 params[:task][:parameters_json_string])
    # 或者如果它们是 task[parameters][target_point_x] 形式，则需要更复杂的 permit
    base_params
  end

  # 定义更新任务时允许的参数
  def task_params_for_update
    # 用户通常只能修改有限的字段，如 priority, scheduled_at, 或任务特定的参数 (通过progress_details)
    params.require(:task).permit(
      :priority, :scheduled_at, :book_id, # 示例，根据实际需求调整
      :source_slot_id, :target_slot_id, :map_id
      # 不允许修改 :task_type, :user_id, :status (除非特定逻辑)
    )
    # progress_details 中的 parameters 部分在 action 中单独处理
  end

  # 检查用户是否有权限创建特定类型的任务
  def check_task_creation_permissions
    # new action 时，task_type可能来自params
    task_type_to_check = params[:task_type] || params.dig(:task, :task_type)

    if task_type_to_check.present?
      admin_only_tasks = [ :map_build_auto, :inventory_scan_and_relocate, :load_map ] # 示例
      if admin_only_tasks.include?(task_type_to_check.to_sym) && !Current.user.admin?
        redirect_to tasks_path, alert: t("tasks.notices.admin_required_for_type", type: t("tasks.types.#{task_type_to_check}"))
      end
    end
  end

  # 判断任务类型是否需要机器人执行
  def needs_robot_execution?(task_type_symbol)
    # task_type_symbol 是字符串或符号
    robot_tasks = [ :map_build_auto, :navigation_to_point, :fetch_book_to_transfer,
                   :return_book_from_transfer, :inventory_scan_and_relocate, :load_map ]
    robot_tasks.include?(task_type_symbol.to_sym)
  end

  # 从 Controller 向 ROS 广播指令的辅助方法
  def broadcast_task_to_ros_from_controller(command_type, task)
    # 与 RobotTaskChannel 中的 broadcast_task_to_ros 逻辑类似
    ros_parameters = task.fetch_parameters

    if task.type_load_map? && task.map
      ros_parameters[:map_id_in_rails] = task.map.id
      ros_parameters[:map_name] ||= task.map.name
      config_url = task.map.map_config_access_url # 使用模型方法获取URL
      image_url = task.map.map_image_access_url
      unless config_url && image_url
          task.update(status: :failed, result_data: (task.result_data || {}).merge(error: "地图文件URL无法从Controller生成。"))
          return
      end
      ros_parameters[:map_config_url] = config_url
      ros_parameters[:map_image_url] = image_url
    elsif task.type_navigation_to_point?
      active_map = RobotStatus.current.active_map
      if active_map
        ros_parameters[:active_map_name_in_rails] = active_map.name
      else
        task.update(status: :failed, result_data: (task.result_data || {}).merge(error: "导航任务需要活动地图，但未设置。"))
        return
      end
    end

    instruction_to_ros = {
      command_type: command_type,
      task: { id: task.id, type: task.task_type.to_s.upcase, parameters: ros_parameters }
    }
    ActionCable.server.broadcast("ros_comms_channel", instruction_to_ros)
    logger.info "[TasksController] Broadcasted #{command_type} for Task #{task.id} to 'ros_comms_channel'."
    logger.debug "[TasksController] ROS instruction payload: #{instruction_to_ros.to_json}"
  end
end
