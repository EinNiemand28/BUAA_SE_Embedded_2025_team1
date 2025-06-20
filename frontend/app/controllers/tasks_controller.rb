# app/controllers/tasks_controller.rb
class TasksController < ApplicationController
  before_action :authenticate
  before_action :set_task, except: [ :index, :new, :create ]

  def index
    @tasks = if Current.user.admin?
      Task.all.includes(:user, :book, :source_slot, :target_slot, :map)
    else
      Current.user.tasks.includes(:book, :source_slot, :target_slot, :map)
    end

    # 添加筛选逻辑
    if params[:status].present?
      @tasks = @tasks.where(status: params[:status])
    end

    if params[:task_type].present?
      @tasks = @tasks.where(task_type: params[:task_type])
    end

    # 用户筛选（仅管理员可用）
    if params[:user_id].present? && Current.user.admin?
      @tasks = @tasks.where(user_id: params[:user_id])
    end

    # 搜索功能
    if params[:search].present?
      search_term = "%#{params[:search]}%"
      @tasks = @tasks.where(
        "id::text ILIKE ? OR task_type ILIKE ? OR status ILIKE ? OR details::text ILIKE ?",
        search_term, search_term, search_term, search_term
      )
    end

    @tasks = @tasks.order(created_at: :desc)
    # 可以根据需要添加分页
    # @tasks = @tasks.page(params[:page]).per(12)
  end

  def show
    @logs = SystemLog.where(task_id: @task.id).order(created_at: :desc)
    # 确保前端JS可以访问到 task_id (如果TaskUpdateChannel需要)
    # 例如，在视图中: <div data-task-detail-id="<%= @task.id %>">
  end

  def new
    @task = Task.new
  end

  def create
    nil
    # 不允许直接由表单创建
  end

  def edit
    nil
  end

  def update
    nil
  end

  def destroy
    # 增加权限检查和任务状态检查
    unless Current.user.admin? || @task.user_id == Current.user.id
      redirect_to tasks_path, alert: t("common.not_authorized") and return
    end
    unless @task.status_failed? || @task.status_completed? || @task.status_cancelled?
      redirect_to @task, alert: t("tasks.notices.cannot_delete") and return
    end

    @task.destroy
    SystemLog.log(:task_event, :warning, "任务 ##{@task.id} (#{task.task_type}) 被用户 #{Current.user.id} 删除。",  "TasksController#destroy", { user_id: Current.user.id, task_id: @task.id })
    redirect_to tasks_path, notice: t("tasks.notices.deleted_successfully")
  end

  private

  def set_task
    @task = Task.find(params[:id])
  rescue ActiveRecord::RecordNotFound
    redirect_to tasks_path, alert: t("tasks.notices.not_found")
  end
end
