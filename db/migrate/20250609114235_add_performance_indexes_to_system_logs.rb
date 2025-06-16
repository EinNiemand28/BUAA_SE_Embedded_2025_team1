class AddPerformanceIndexesToSystemLogs < ActiveRecord::Migration[7.2]
  def change
    # 为最常见的查询添加复合索引
    # 时间戳 + 日志类型的复合索引（常用于分页和过滤）
    add_index :system_logs, [ :created_at, :log_type ],
              name: 'index_system_logs_on_created_at_and_log_type'

    # 时间戳 + 严重程度的复合索引
    add_index :system_logs, [ :created_at, :severity ],
              name: 'index_system_logs_on_created_at_and_severity'

    # 用于消息搜索的索引（PostgreSQL专用）
    if ActiveRecord::Base.connection.adapter_name.downcase == 'postgresql'
      # 为消息字段添加GIN索引，优化ILIKE查询（不使用 CONCURRENTLY）
      execute "CREATE INDEX IF NOT EXISTS index_system_logs_on_message_gin ON system_logs USING gin(to_tsvector('simple', message))"
    end
  end

  def down
    remove_index :system_logs, name: 'index_system_logs_on_created_at_and_log_type'
    remove_index :system_logs, name: 'index_system_logs_on_created_at_and_severity'

    if ActiveRecord::Base.connection.adapter_name.downcase == 'postgresql'
      execute "DROP INDEX IF EXISTS index_system_logs_on_message_gin"
    end
  end
end
