class Bookshelf < ApplicationRecord
  # 关联
  has_many :slots, dependent: :destroy # 书架删除时，其下的插槽也应删除

  # 验证
  validates :code, presence: true, uniqueness: true, length: { maximum: 10 }
  validates :name, length: { maximum: 50 }
  validates :center_x, presence: true, numericality: true
  validates :center_y, presence: true, numericality: true
  validates :length, presence: true, numericality: { greater_than: 0 }
  validates :width, presence: true, numericality: { greater_than: 0 }
  validates :height, presence: true, numericality: { greater_than: 0 }
  validates :levels, presence: true, numericality: { only_integer: true, greater_than: 0 }
  validates :slots_per_level, presence: true, numericality: { only_integer: true, greater_than: 0 }
  validates :is_transit_station, inclusion: { in: [ true, false ] }

  # 回调
  after_create :generate_slots # 在创建书架后自动生成插槽

  # scope
  scope :transit_stations, -> { where(is_transit_station: true) }
  scope :normal_shelves, -> { where(is_transit_station: false) }

  # 可以添加一个方法来计算书架的总插槽数
  def total_slots
    levels * slots_per_level
  end

  def navigation_coordinates(distance: 0.75)
    shelf_orientation = self.orientation || 0 # 如果朝向为空，默认为0
    front_direction = shelf_orientation + Math::PI # 书架前方方向（180度）
    half_width = self.width / 2.0

    front_x = self.center_x + half_width * Math.cos(shelf_orientation)
    front_y = self.center_y + half_width * Math.sin(shelf_orientation)

    nav_x = front_x + distance * Math.cos(shelf_orientation)
    nav_y = front_y + distance * Math.sin(shelf_orientation)
    nav_z = 0.0

    { x: nav_x.round(4), y: nav_y.round(4), z: nav_z.round(4), oz: front_direction.round(4) }
  end

  private

  def generate_slots
    # 实现插槽生成逻辑 (将在下一步完成)
    # 确保 levels 和 slots_per_level 存在且有效
    return unless levels.to_i > 0 && slots_per_level.to_i > 0

    # 清空可能已存在的插槽（确保幂等性，尽管 after_create 只运行一次）
    # self.slots.destroy_all # 通常不需要在 after_create 中，但保留注释以备不时之需

    slot_list = []
    (0...levels).each do |level_index|
      (0...slots_per_level).each do |row_index|
        # 计算相对坐标 (这是一个简化的示例，可能需要根据具体书架调整)
        # 假设书架沿 Y 轴放置，长度在 X 轴方向，宽度在 Y 轴方向
        # 插槽从左到右 (X增加)，从下到上 (Z增加) 编号
        slot_width = self.length / slots_per_level
        slot_depth_center = 0 # 假设书放在书架中线
        slot_height_center = (self.bottom_clearance || 0) + (self.level_height || (self.height - (self.bottom_clearance || 0)) / levels) * (level_index + 0.5)

        relative_x = -self.length / 2.0 + slot_width * (row_index + 0.5)
        relative_y = slot_depth_center # 假设插槽中心在书架宽度方向的中心线上
        relative_z = slot_height_center

        slot_list << {
          bookshelf_id: self.id,
          level: level_index,
          row: row_index,
          relative_x: relative_x.round(4),
          relative_y: relative_y.round(4),
          relative_z: relative_z.round(4),
          created_at: Time.current,
          updated_at: Time.current
        }
      end
    end
    # 批量插入以提高性能
    Slot.insert_all(slot_list) if slot_list.any?
    Rails.logger.info "Generated #{slot_list.size} slots for Bookshelf ##{self.id} (#{self.code})"
  rescue => e
    Rails.logger.error "Failed to generate slots for Bookshelf ##{self.id}: #{e.message}"
    # 可以考虑是否需要回滚书架的创建或记录错误
  end
end
