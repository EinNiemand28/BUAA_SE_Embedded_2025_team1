# app/models/map.rb
class Map < ApplicationRecord
  # Active Storage 关联
  has_one_attached :map_config  # ROS地图的.yaml配置文件
  has_one_attached :map_image   # ROS地图的.pgm图像文件 (或其他格式如.png)
  has_one_attached :thumbnail   # 地图缩略图，用于UI展示

  # 关联关系
  belongs_to :created_by_user, class_name: "User", foreign_key: "created_by_user_id" # 创建此地图的用户
  # 一个地图可能由一个特定的建图任务创建
  belongs_to :creating_task, class_name: "Task", foreign_key: "task_id", optional: true # 注意: schema.rb 中 task 表有 map_id, Map表没有 task_id. 这需要调整。
  # 通常是 Task has_one :map 或 Task belongs_to :map
  # 这里假设 Task belongs_to :map, 所以 Map has_one :task_that_created_it (或类似)
  # 或者，如果 Task.map_id 指向这个Map，那么 Map has_one :task
  has_one :task # 指向创建此地图的那个任务 (如果 Task.map_id 指向此 Map 记录)

  # 验证
  validates :name, presence: { message: "地图名称不能为空" },
                   uniqueness: { case_sensitive: false, message: "地图名称必须唯一" }
  validates :created_by_user_id, presence: { message: "必须关联创建用户" }
  # 验证 ActiveStorage 附件的存在性 (可选，但推荐在某些场景下)
  # validates :map_config, attached: true, on: :update, if: -> { !is_active? } # 例如，非活动地图可以没有文件，但激活前必须有
  # validates :map_image, attached: true, on: :update, if: -> { !is_active? }


  # 作用域
  scope :active_maps, -> { where(is_active: true) } # 注意：is_active 字段在你的 schema.rb 中

  # 回调
  # 当 is_active 变为 true 时，确保其他地图的 is_active 都为 false
  before_save :ensure_only_one_active_map, if: -> { will_save_change_to_is_active? && is_active? }
  # 异步生成缩略图 (如果 map_image 更新了)
  after_commit :generate_thumbnail_later, on: [ :create, :update ], if: -> { map_image.attached? && (saved_change_to_active_storage_blobs? || map_image.blob.created_at > 1.minute.ago) }


  # 类方法
  # 获取当前活动的地图 (理论上只有一个)
  def self.active_map
    active_maps.first
  end

  # 实例方法

  # 激活此地图（并自动停用其他地图）
  # 注意：这个方法只是在数据库层面激活。实际让机器人加载此地图需要创建一个 :load_map 类型的 Task。
  def activate_in_db!
    return true if is_active? # 如果已经是激活状态，则不执行任何操作

    Map.transaction do
      # 先将所有其他地图设置为非激活
      Map.where.not(id: self.id).update_all(is_active: false)
      # 然后将当前地图设置为激活
      self.update!(is_active: true) # 使用 update! 确保如果失败则抛出异常
    end
  rescue ActiveRecord::RecordInvalid => e
    errors.add(:base, "激活地图失败: #{e.message}")
    false
  end

  # 停用此地图
  def deactivate_in_db!
    update(is_active: false) if is_active?
  end

  # 获取地图配置文件的可访问URL (用于ROS节点下载)
  def map_config_access_url
    return nil unless map_config.attached?
    # 考虑使用 signed_id 或直接的 blob_url，取决于你的存储服务和安全策略
    # Rails.application.routes.url_helpers.rails_blob_url(map_config, disposition: "attachment", only_path: false)
    # 为了让ROS能直接访问，通常需要一个无认证的、短时效的URL或永久公共URL（如果安全允许）
    # 如果你的存储是本地public，可以直接用路径。如果是云存储，需要生成可访问URL。
    # 暂时返回一个可以直接访问的 blob URL
    begin
        base_url = Rails.application.config.action_controller.asset_host || Rails.application.routes.url_helpers.root_url(protocol: "http")
        Rails.application.routes.url_helpers.rails_blob_url(map_config, host: base_url)
    rescue => e
        Rails.logger.error "生成 map_config_access_url 失败: #{e.message}"
        nil
    end
  end

  # 获取地图图像文件的可访问URL
  def map_image_access_url
    return nil unless map_image.attached?
    begin
        base_url = Rails.application.config.action_controller.asset_host || Rails.application.routes.url_helpers.root_url(protocol: "http")
        Rails.application.routes.url_helpers.rails_blob_url(map_image, host: base_url)
    rescue => e
        Rails.logger.error "生成 map_image_access_url 失败: #{e.message}"
        nil
    end
  end

  # 获取缩略图URL (用于Web界面显示)
  def display_thumbnail_url
    if thumbnail.attached?
      Rails.application.routes.url_helpers.url_for(thumbnail)
    elsif map_image.attached? && map_image.variable? # 检查图像是否可以处理（例如是图片类型）
      begin
        Rails.application.routes.url_helpers.url_for(map_image.variant(resize_to_limit: [ 200, 200 ]).processed)
      rescue ActiveStorage::InvariableError, MiniMagick::Error => e # MiniMagick::Error for non-image files
        Rails.logger.warn "无法为 Map #{id} 的 map_image 生成缩略图: #{e.message}"
        nil # 或者返回一个占位符图像URL
      end
    else
      nil # 或者一个占位符图像URL
    end
  end

  private

  # 回调方法：确保数据库中只有一个地图的 is_active 为 true
  def ensure_only_one_active_map
    # SQL语句直接更新，效率更高
    Map.where.not(id: self.id).update_all(is_active: false)
  end

  # 回调方法：异步任务来生成缩略图
  def generate_thumbnail_later
    # 确保map_image已保存且是图片类型
    return unless map_image.attached? && map_image.content_type.start_with?("image/")

    # 可以使用 ActiveJob 来异步处理缩略图生成，避免阻塞请求
    # GenerateMapThumbnailJob.perform_later(self.id)
    # 这里为了简单，我们同步尝试生成，但在生产环境中强烈建议异步
    generate_thumbnail_now
  end

  # （辅助方法，实际中应放入Job）同步生成缩略图
  def generate_thumbnail_now
    return unless map_image.attached? && map_image.content_type.start_with?("image/")

    begin
      # 使用 image_processing gem 来生成缩略图
      processed_variant = map_image.variant(resize_to_limit: [ 300, 300 ], format: :png).processed
      # 将处理后的变体附加到 thumbnail 字段
      # 需要将变体下载到临时文件再附加，或者直接操作blob（如果支持）

      # 方案1：下载并重新附加 (简单但可能效率稍低)
      # temp_file = Tempfile.new(["thumbnail_#{self.id}", ".png"])
      # temp_file.binmode
      # processed_variant.download { |chunk| temp_file.write(chunk) }
      # temp_file.flush
      # self.thumbnail.attach(io: temp_file, filename: "#{self.name}_thumbnail.png", content_type: "image/png")
      # temp_file.close
      # temp_file.unlink

      # 方案2：直接操作blob (如果ActiveStorage支持这种操作，或者image_processing有更好的方式)
      # 查阅 image_processing 和 ActiveStorage 文档获取最佳实践
      # 临时的简单实现：
      if processed_variant.service_url # 如果可以直接获取URL
        # self.thumbnail.attach(io: URI.open(processed_variant.service_url), filename: ... )
        # 更好的方式是如果 variant.blob 能直接用于 attach
      end
      # 注意：上述缩略图生成逻辑可能需要根据你的 ActiveStorage 服务（local, S3 等）和
      # image_processing 的具体用法进行调整。
      # Map模型文件中的 process_map_files_with_active_storage 方法中的缩略图逻辑可能更直接。
      # 确保不重复生成。

      Rails.logger.info "Map ##{self.id}: 缩略图已（尝试）生成/更新。"
    rescue StandardError => e
      Rails.logger.error "Map ##{self.id}: 生成缩略图失败: #{e.message}"
    end
  end

  # 检查 Active Storage blobs 是否有变化 (用于 after_commit 回调)
  def saved_change_to_active_storage_blobs?
    saved_changes.keys.any? { |k| k.match?(/_blob_id$/) }
  end
end
