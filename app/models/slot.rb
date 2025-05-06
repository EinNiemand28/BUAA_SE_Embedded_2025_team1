class Slot < ApplicationRecord
  belongs_to :bookshelf

  # 关联：一个插槽当前最多放一本书 (实际占用)
  has_one :current_book, class_name: 'Book', foreign_key: 'current_slot_id', dependent: :nullify
  
  # 关联：一个插槽可能是某本书的预定位置
  has_one :intended_book, class_name: 'Book', foreign_key: 'intended_slot_id', dependent: :nullify
  
  # 验证
  validates :row, presence: true, numericality: { only_integer: true, greater_than_or_equal_to: 0 }
  validates :level, presence: true, numericality: { only_integer: true, greater_than_or_equal_to: 0 }
  validates :relative_x, presence: true, numericality: true
  validates :relative_y, presence: true, numericality: true
  validates :relative_z, presence: true, numericality: true
  validates :is_occupied, inclusion: { in: [true, false] }
  
  # 复合唯一性验证（数据库层面已有索引，模型层面可加可不加，为保险起见加上）
  validates :row, uniqueness: { scope: [:bookshelf_id, :level], message: "在同一书架的同一层中已存在" }

  # 更新占用状态的回调或方法 (可选)
  # 可以在 Book 的 current_slot_id 改变时触发更新 Slot 的 is_occupied
  # 或者在查询 Slot 时动态判断
  def update_occupied_status
    update(is_occupied: current_book.present?)
  end

  # 计算插槽在世界坐标系中的绝对坐标
  # 注意：这需要书架的中心坐标和朝向
  def absolute_coordinates
    return nil unless bookshelf # 确保关联存在

    shelf_x = bookshelf.center_x
    shelf_y = bookshelf.center_y
    shelf_orientation = bookshelf.orientation || 0 # 如果朝向为空，默认为0

    # 将相对坐标根据书架朝向进行旋转和平移
    # cos_o = Math.cos(shelf_orientation)
    # sin_o = Math.sin(shelf_orientation)

    # X_abs = shelf_x + relative_x * cos_o - relative_y * sin_o
    # Y_abs = shelf_y + relative_x * sin_o + relative_y * cos_o
    # Z_abs = relative_z # Z坐标（高度）通常是绝对的，不需要旋转
    
    # 使用更健壮的TF转换逻辑（如果适用）或简单的旋转矩阵
    # 这里使用基本旋转公式
    rotated_x = relative_x * Math.cos(shelf_orientation) - relative_y * Math.sin(shelf_orientation)
    rotated_y = relative_x * Math.sin(shelf_orientation) + relative_y * Math.cos(shelf_orientation)

    abs_x = shelf_x + rotated_x
    abs_y = shelf_y + rotated_y
    abs_z = relative_z

    { x: abs_x.round(4), y: abs_y.round(4), z: abs_z.round(4) }
  end
end
