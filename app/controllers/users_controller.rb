# app/controllers/users_controller.rb
class UsersController < ApplicationController
  layout "application" # 确保使用主布局
  before_action :authenticate
  before_action :require_admin!     # 确保只有管理员能访问
  before_action :set_user, only: [ :edit, :update, :destroy ]

  def index
    @users = User.order(created_at: :desc) # 按创建时间排序
    if params[:query].present?
      query_term = "%#{params[:query].downcase}%" # 小写以实现不区分大小写搜索
      @users = @users.where("LOWER(username) LIKE :query OR LOWER(email) LIKE :query", query: query_term)
    end
    # @users = @users.page(params[:page]).per(15) # Kaminari分页示例
  end

  def edit
    # @user 由 set_user 回调设置
  end

  def update
    # 管理员可以更新用户的角色、用户名、邮箱
    # 密码更改应通过用户的密码重置流程或个人资料编辑进行
    if @user.update(user_params_for_admin_update)
      SystemLog.log(:user_action, :info, "管理员 #{Current.user.id} 更新了用户 ##{@user.id} (#{@user.username}) 的信息。",
                    "UsersController#update", { admin_user_id: Current.user.id, target_user_id: @user.id })
      redirect_to users_path, notice: t("users.notices.updated_successfully", username: @user.username)
    else
      render :edit, status: :unprocessable_entity
    end
  end

  def destroy
    # 防止管理员删除自己
    if @user == Current.user
      redirect_to users_path, alert: t("users.notices.cannot_delete_self")
    else
      # 安全删除：考虑任务等关联数据的处理策略
      # 例如，如果用户有很多重要数据，可能不允许删除，或标记为deactivated
      username_for_log = @user.username # 在销毁前记录用户名
      user_id_for_log = @user.id

      if @user.destroy
        SystemLog.log(:user_action, :warning, "管理员 #{Current.user.id} 删除了用户 ##{user_id_for_log} (#{username_for_log})。",
                      "UsersController#destroy", { admin_user_id: Current.user.id, target_user_id: user_id_for_log })
        redirect_to users_path, notice: t("users.notices.deleted_successfully", username: username_for_log)
      else
        # 如果有 before_destroy 回调阻止了删除 (例如因为有关联数据)
        redirect_to users_path, alert: t("users.notices.delete_failed", username: username_for_log, errors: @user.errors.full_messages.join(", "))
      end
    end
  end

  private

  def set_user
    @user = User.find(params[:id])
  rescue ActiveRecord::RecordNotFound
    redirect_to users_path, alert: t("users.notices.not_found")
  end

  # 管理员更新用户时允许的参数
  def user_params_for_admin_update
    # 允许管理员修改用户名、邮箱、角色和验证状态
    # 不允许直接通过此表单修改密码
    params.require(:user).permit(:username, :email, :role, :verified)
  end

  def require_admin!
    unless Current.user&.admin?
      redirect_to root_path, alert: t("common.not_authorized")
    end
  end
end
