class ProfilesController < ApplicationController
  # 使用主应用程序布局
  layout "application"

  def show
    # authenticate 过滤器已在 ApplicationController 中确保用户登录
    @user = Current.session.user
  end
end 