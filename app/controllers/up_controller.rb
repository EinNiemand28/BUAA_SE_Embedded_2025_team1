class UpController < ApplicationController
  # 跳过所有过滤器和认证
  skip_before_action :verify_authenticity_token, :authenticate_user!, raise: false
  
  def index
    render plain: "OK", status: 200
  end
end