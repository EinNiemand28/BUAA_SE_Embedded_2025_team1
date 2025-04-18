class UpController < ApplicationController
  # 完全跳过所有Rails过滤器
  skip_forgery_protection
  skip_before_action :authenticate_user!, raise: false
  skip_before_action :verify_authenticity_token, raise: false
  
  def index
    render plain: "OK", status: 200
  end
end