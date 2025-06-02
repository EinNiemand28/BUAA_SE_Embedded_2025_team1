module TasksHelper
  # 根据任务类型返回对应的图标
  def task_type_icon(task_type)
    case task_type.to_s
    when 'map_build_auto'
      content_tag(:svg, class: "w-5 h-5 text-purple-600", fill: "none", stroke: "currentColor", viewBox: "0 0 24 24") do
        content_tag(:path, "", "stroke-linecap": "round", "stroke-linejoin": "round", "stroke-width": "2", 
                    d: "M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-1.447-.894L15 4m0 13V4m0 0L9 7")
      end
    when 'navigation_to_point'
      content_tag(:svg, class: "w-5 h-5 text-blue-600", fill: "none", stroke: "currentColor", viewBox: "0 0 24 24") do
        content_tag(:path, "", "stroke-linecap": "round", "stroke-linejoin": "round", "stroke-width": "2", 
                    d: "M17.657 16.657L13.414 20.9a1.998 1.998 0 01-2.827 0l-4.244-4.243a8 8 0 1111.314 0z") +
        content_tag(:path, "", "stroke-linecap": "round", "stroke-linejoin": "round", "stroke-width": "2", 
                    d: "M15 11a3 3 0 11-6 0 3 3 0 016 0z")
      end
    when 'fetch_book_to_transfer', 'return_book_from_transfer'
      content_tag(:svg, class: "w-5 h-5 text-green-600", fill: "none", stroke: "currentColor", viewBox: "0 0 24 24") do
        content_tag(:path, "", "stroke-linecap": "round", "stroke-linejoin": "round", "stroke-width": "2", 
                    d: "M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.747 0 3.332.477 4.5 1.253v13C19.832 18.477 18.247 18 16.5 18c-1.746 0-3.332.477-4.5 1.253")
      end
    when 'inventory_scan_and_relocate'
      content_tag(:svg, class: "w-5 h-5 text-orange-600", fill: "none", stroke: "currentColor", viewBox: "0 0 24 24") do
        content_tag(:path, "", "stroke-linecap": "round", "stroke-linejoin": "round", "stroke-width": "2", 
                    d: "M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z")
      end
    when 'load_map'
      content_tag(:svg, class: "w-5 h-5 text-indigo-600", fill: "none", stroke: "currentColor", viewBox: "0 0 24 24") do
        content_tag(:path, "", "stroke-linecap": "round", "stroke-linejoin": "round", "stroke-width": "2", 
                    d: "M7 16a4 4 0 01-.88-7.903A5 5 0 1115.9 6L16 6a5 5 0 011 9.9M9 19l3 3m0 0l3-3m-3 3V10")
      end
    else
      content_tag(:svg, class: "w-5 h-5 text-gray-600", fill: "none", stroke: "currentColor", viewBox: "0 0 24 24") do
        content_tag(:path, "", "stroke-linecap": "round", "stroke-linejoin": "round", "stroke-width": "2", 
                    d: "M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 100 4m0-4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 100 4m0-4v2m0-6V4")
      end
    end
  end

  # 根据任务类型返回对应的背景色类
  def task_type_icon_bg(task_type)
    case task_type.to_s
    when 'map_build_auto'
      "bg-purple-50"
    when 'navigation_to_point'
      "bg-blue-50"
    when 'fetch_book_to_transfer', 'return_book_from_transfer'
      "bg-green-50"
    when 'inventory_scan_and_relocate'
      "bg-orange-50"
    when 'load_map'
      "bg-indigo-50"
    else
      "bg-gray-50"
    end
  end

  # 根据任务状态返回对应的徽章
  def task_status_badge(status)
    case status.to_s
    when 'pending'
      content_tag(:span, t("activerecord.attributes.task.statuses.pending"), 
                  class: "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800")
    when 'queued'
      content_tag(:span, t("activerecord.attributes.task.statuses.queued"), 
                  class: "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-yellow-100 text-yellow-800")
    when 'processing'
      content_tag(:span, t("activerecord.attributes.task.statuses.processing"), 
                  class: "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-blue-100 text-blue-800")
    when 'paused'
      content_tag(:span, t("activerecord.attributes.task.statuses.paused"), 
                  class: "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-orange-100 text-orange-800")
    when 'completed'
      content_tag(:span, t("activerecord.attributes.task.statuses.completed"), 
                  class: "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800")
    when 'failed'
      content_tag(:span, t("activerecord.attributes.task.statuses.failed"), 
                  class: "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-red-100 text-red-800")
    when 'cancelling'
      content_tag(:span, t("activerecord.attributes.task.statuses.cancelling"), 
                  class: "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-yellow-100 text-yellow-800")
    when 'cancelled'
      content_tag(:span, t("activerecord.attributes.task.statuses.cancelled"), 
                  class: "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800")
    else
      content_tag(:span, status.to_s.humanize, 
                  class: "inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800")
    end
  end

  # 格式化任务持续时间
  def format_task_duration(task)
    return t('tasks.show.not_started') unless task.started_at
    
    end_time = task.completed_at || Time.current
    duration = end_time - task.started_at
    
    if duration < 60
      "#{duration.to_i}秒"
    elsif duration < 3600
      "#{(duration / 60).to_i}分钟"
    else
      hours = (duration / 3600).to_i
      minutes = ((duration % 3600) / 60).to_i
      "#{hours}小时#{minutes}分钟"
    end
  end

  # 获取任务进度百分比
  def task_progress_percentage(task)
    return 0 unless task.progress_details.present?
    
    if task.progress_details['progress_percentage']
      task.progress_details['progress_percentage'].to_i
    elsif task.status_completed?
      100
    elsif task.status_processing?
      50 # 默认进行中显示50%
    else
      0
    end
  end
end
