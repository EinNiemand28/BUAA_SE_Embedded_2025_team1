# 配置 Heroicon gem，使其在视图中可用
Rails.application.config.after_initialize do
  # 将 heroicon 方法暴露为辅助方法，使其在所有视图中可用
  ActionView::Base.include Heroicon::Engine.helpers
end
