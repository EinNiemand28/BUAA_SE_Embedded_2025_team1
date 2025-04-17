class RegistrationsController < ApplicationController
  layout "authentication"

  skip_before_action :authenticate

  def new
    @user = User.new
  end

  def create
    @user = User.new(user_params.merge(verified: true))

    if @user.save
      session_record = @user.sessions.create!
      cookies.signed.permanent[:session_token] = { value: session_record.id, httponly: true }

      redirect_to root_path, notice: t('.signed_up_successfully', default: "Welcome! You have signed up successfully")
    else
      render :new, status: :unprocessable_entity
    end
  end

  private
    def user_params
      params.require(:user).permit(:email, :username, :password, :password_confirmation)
    end
end
