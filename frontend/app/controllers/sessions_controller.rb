class SessionsController < ApplicationController
  layout "authentication", only: [:new, :create]

  skip_before_action :authenticate, only: %i[ new create ]

  before_action :set_session, only: :destroy

  def index
    @sessions = Current.user.sessions.order(created_at: :desc)
  end

  def new
    @login_hint = params[:login_hint]
  end

  def create
    login_param = params[:login].downcase
    password_param = params[:password]

    user = User.find_by(email: login_param) || User.find_by(username: params[:login])

    if user && user.authenticate(password_param)
      @session = user.sessions.create!(
        user_agent: Current.user_agent, 
        ip_address: Current.ip_address
      )
      cookies.signed.permanent[:session_token] = { value: @session.id, httponly: true }
      redirect_to root_path, notice: t('.signed_in_successfully')
    else
      redirect_to sign_in_path(login_hint: params[:login]), alert: t('.invalid_login_or_password')
    end
  end

  def destroy
    @session.destroy
    redirect_to(sign_in_path, notice: t('.signed_out_successfully'))
  end

  private
    def set_session
      @session = Current.user.sessions.find(params[:id])
    end
end
