class SystemLogsController < ApplicationController
  before_action :authenticate
  before_action :require_admin!

  def index
    # 使用新的 scope 而不是 unscoped
    @logs = SystemLog.includes(:user, :task, :book)

    # --- 过滤 ---
    # 类型过滤
    if params[:log_type].present? && SystemLog.log_types.key?(params[:log_type].to_s.downcase)
      @logs = @logs.where(log_type: params[:log_type].to_s.downcase)
    end
    # 严重程度过滤
    if params[:severity].present? && SystemLog.severities.key?(params[:severity].to_s.downcase)
      @logs = @logs.where(severity: params[:severity].to_s.downcase)
    end
    # 来源过滤 (精确匹配)
    @logs = @logs.where(source: params[:source]) if params[:source].present?
    # 消息内容模糊搜索 - 修复PostgreSQL兼容性
    if params[:message_contains].present?
      # 使用 ILIKE 代替 LIKE 来支持不区分大小写的搜索
      if ActiveRecord::Base.connection.adapter_name.downcase == 'postgresql'
        @logs = @logs.where("message ILIKE ?", "%#{params[:message_contains]}%")
      else
      @logs = @logs.where("message LIKE ?", "%#{params[:message_contains]}%")
      end
    end
    # 关联对象ID过滤
    @logs = @logs.where(user_id: params[:user_id]) if params[:user_id].present?
    @logs = @logs.where(task_id: params[:task_id]) if params[:task_id].present?
    @logs = @logs.where(book_id: params[:book_id]) if params[:book_id].present?
    # 时间范围过滤
    if params[:start_date].present?
      begin
      @logs = @logs.where("created_at >= ?", Date.parse(params[:start_date]).beginning_of_day)
      rescue Date::Error
        # 忽略无效日期
      end
    end
    if params[:end_date].present?
      begin
      @logs = @logs.where("created_at <= ?", Date.parse(params[:end_date]).end_of_day)
      rescue Date::Error
        # 忽略无效日期
      end
    end

    # 使用显式的排序scope
    @logs = @logs.latest_first
    
    # 分页（调整每页数量）
    @logs = @logs.page(params[:page]).per(20)

    # 用于过滤表单的选项
    @log_type_options = SystemLog.log_types.keys.map { |type| [ t("system_logs.log_types.#{type}", default: type.humanize), type ] }
    @severity_options = SystemLog.severities.keys.map { |sev| [ t("system_logs.severities.#{sev}", default: sev.humanize), sev ] }
    
    # 获取source选项
    @source_options = SystemLog.distinct.pluck(:source).compact.sort
  end

  def show
    @log = SystemLog.find(params[:id])
  end

  private

  def require_admin!
    unless Current.user&.admin?
      redirect_to root_path, alert: t("common.not_authorized")
    end
  end
end
