# app/models/map.rb
class Map < ApplicationRecord
  have_one_attached :map_image
  # 关联关系
  belongs_to :created_by_user, class_name: "User", foreign_key: "created_by_user_id"

  # 验证
  validates :name, presence: { message: "地图名称不能为空" },
                   uniqueness: { case_sensitive: false, message: "地图名称必须唯一" }
  validates :created_by_user_id, presence: { message: "必须关联创建用户" }

  # 作用域
  scope :active_maps, -> { where(is_active: true) } # 注意：is_active 字段在你的 schema.rb 中

  # 回调
  # 当 is_active 变为 true 时，确保其他地图的 is_active 都为 false
  before_save :ensure_only_one_active_map, if: -> { will_save_change_to_is_active? && is_active? }

  # 类方法
  # 获取当前活动的地图 (理论上只有一个)
  def self.active_map
    active_maps.first
  end

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

  private

  # 回调方法：确保数据库中只有一个地图的 is_active 为 true
  def ensure_only_one_active_map
    # SQL语句直接更新，效率更高
    Map.where.not(id: self.id).update_all(is_active: false)
  end
end
