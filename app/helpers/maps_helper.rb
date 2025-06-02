module MapsHelper
  # 地图状态徽章样式
  def map_status_badge(is_active)
    if is_active
      "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800"
    else
      "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800"
    end
  end

  # 地图状态文本
  def map_status_text(is_active)
    is_active ? t('maps.status.active') : t('maps.status.inactive')
  end

  # 地图卡片边框样式
  def map_card_class(is_active)
    base_class = "bg-white overflow-hidden shadow-lg rounded-lg border border-gray-200 hover:shadow-xl transition-shadow duration-200"
    if is_active
      "#{base_class} ring-2 ring-green-500"
    else
      base_class
    end
  end

  # 检查地图是否可以删除
  def map_can_be_deleted?(map)
    return false if map.is_active?
    !Task.where(map_id: map.id).where.not(status: [:completed, :failed, :cancelled]).exists?
  end

  # 地图文件大小格式化
  def format_map_file_size(bytes)
    return "0 B" if bytes.nil? || bytes.zero?
    
    units = %w[B KB MB GB TB]
    index = (Math.log(bytes) / Math.log(1024)).floor
    size = bytes / (1024.0 ** index)
    
    "#{size.round(2)} #{units[index]}"
  end

  # 地图相关任务计数
  def map_tasks_count(map)
    Task.where(map: map).count
  end

  # 地图最近任务
  def map_recent_tasks(map, limit = 5)
    Task.where(map: map).order(created_at: :desc).limit(limit)
  end

  # 地图激活按钮样式
  def map_activate_button_class
    "text-xs bg-green-600 text-white px-2 py-1 rounded hover:bg-green-700 transition-colors"
  end

  # 地图删除按钮样式
  def map_delete_button_class
    "text-xs text-red-600 hover:text-red-700 px-2 py-1 border border-red-300 rounded hover:bg-red-50 transition-colors"
  end

  # 检查用户是否可以管理地图
  def can_manage_maps?
    Current.session&.user&.admin?
  end

  # 地图预览图片URL
  def map_preview_url(map, variant_options = {})
    return nil unless map.map_image.attached?
    
    default_options = { resize_to_limit: [400, 225] }
    options = default_options.merge(variant_options)
    
    url_for(map.map_image.variant(options))
  end

  # 地图数据URL简化显示
  def truncated_map_data_url(url, length = 50)
    return "" if url.blank?
    truncate(url, length: length, omission: "...")
  end
end
