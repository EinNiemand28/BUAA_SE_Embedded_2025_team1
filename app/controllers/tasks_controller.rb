# app/controllers/tasks_controller.rb
class TasksController < ApplicationController
  before_action :authenticate
  before_action :set_task, except: [ :index, :new, :create ]

  def index
    @tasks = if Current.user.admin?
      Task.all.includes(:user, :book, :source_slot, :target_slot, :map).order(created_at: :desc)
    else
      Current.user.tasks.includes(:book, :source_slot, :target_slot, :map).order(created_at: :desc)
    end
    @tasks = @tasks.page(params[:page]).per(10) # Kaminari示例
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
    unless @task.status_failed || @task.status_completed || @task.status_cancelled
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
