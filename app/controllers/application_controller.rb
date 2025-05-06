class ApplicationController < ActionController::Base
  # Only allow modern browsers supporting webp images, web push, badges, import maps, CSS nesting, and CSS :has.
  allow_browser versions: :modern

  before_action :set_current_request_details
  before_action :set_current_session
  before_action :authenticate

  private

    def set_current_session
      if session_record = Session.find_by_id(cookies.signed[:session_token])
        Current.session = session_record
      else
        Current.session = nil
      end
    end

    def authenticate
      unless Current.session
        redirect_to sign_in_path, alert: t('application.authenticate.require_login', default: "请先登录")
      end
    end

    def require_admin
      unless Current.session&.user&.admin?
        redirect_to root_path, alert: t('application.require_admin.unauthorized', default: "您没有权限访问此页面")
      end
    end

    def set_current_request_details
      Current.user_agent = request.user_agent
      Current.ip_address = request.ip
    end
end
