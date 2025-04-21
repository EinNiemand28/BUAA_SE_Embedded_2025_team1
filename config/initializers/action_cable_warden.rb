# config/initializers/action_cable_warden.rb
# Ensure Action Cable connections have access to the Warden session/user

module ActionCable
  module Connection
    class WardenHooks
      def self.before_verifying_user(env)
        # Nothing needed here for typical Warden setup
      end

      def self.after_verifying_user(user)
        # Nothing needed here
      end
    end
  end
end

Rails.application.config.after_initialize do
  ActionCable.server.config.connection_class = -> { ApplicationCable::Connection }

  # Inject Warden::Proxy into the Action Cable connection environment
  # This makes request.env['warden'] available in the Connection class
  module ActionCable
    module Connection
      class Base
        alias_method :old_initialize, :initialize

        def initialize(server, env, coder: ActiveSupport::JSON)
          env['warden'] ||= env['action_dispatch.request.unsigned_session'].fetch('warden.user.user.key', nil) rescue nil
          old_initialize(server, env, coder: coder)
        end
      end
    end
  end
end 