class UsersController < ApplicationController
  # 使用主应用程序布局
  layout "application"
  before_action :require_admin
  before_action :set_user, only: [:edit, :update, :destroy]

  def index
    @users = User.order(id: :asc)
    if params[:query].present?
      query_term = "%#{params[:query]}%"
      # 使用 LOWER() 和 LIKE 实现跨数据库的大小写不敏感搜索
      @users = @users.where("LOWER(username) LIKE LOWER(?) OR LOWER(email) LIKE LOWER(?)", query_term, query_term)
    end
    # 可以添加分页 gem 如 pagy
  end

  def edit
    # @user 由 set_user 设置
  end

  def update
    if @user.update(user_params)
      redirect_to users_path, notice: t('.user_updated_successfully')
    else
      render :edit, status: :unprocessable_entity
    end
  end

  def destroy
    # 防止管理员删除自己
    if @user == Current.session.user
      redirect_to users_path, alert: t('.cannot_delete_self')
    else
      @user.destroy
      redirect_to users_path, notice: t('.user_deleted_successfully')
    end
  end

  private

  def set_user
    @user = User.find(params[:id])
  rescue ActiveRecord::RecordNotFound
    redirect_to users_path, alert: t('.user_not_found')
  end

  def user_params
    # 允许管理员修改用户名、邮箱和角色
    # 不允许直接修改密码
    params.require(:user).permit(:username, :email, :role)
  end
end 