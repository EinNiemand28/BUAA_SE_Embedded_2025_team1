class Book < ApplicationRecord
  # 封面图片关联
  has_one_attached :cover_image
  
  # 关联
  belongs_to :current_slot, class_name: 'Slot', foreign_key: 'current_slot_id', optional: true
  belongs_to :intended_slot, class_name: 'Slot', foreign_key: 'intended_slot_id', optional: true
  
  # 委托 bookshelf 信息给 slot
  delegate :bookshelf, to: :current_slot, allow_nil: true, prefix: :current
  delegate :bookshelf, to: :intended_slot, allow_nil: true, prefix: :intended

  # 枚举定义书籍状态 - 使用整数值
  enum status: { available: 0, borrowed: 1, unavailable: 2, transit: 3 }
  
  # 验证
  validates :isbn, presence: true, uniqueness: true
  validates :title, presence: true
  validates :author, presence: true
  validates :status, presence: true
  # 验证：如果设置了 current_slot，该 slot 不能被其他 book 占用
  validate :current_slot_must_be_available, if: :current_slot_id_changed?
  # 验证：如果设置了 intended_slot，该 slot 不能是其他 book 的 intended_slot
  validate :intended_slot_must_be_available, if: :intended_slot_id_changed?
  
  # 回调: 当 current_slot 改变时，更新旧 slot 和新 slot 的占用状态
  after_save :update_slot_occupancy, if: :saved_change_to_current_slot_id?

  # 搜索方法
  def self.search(query)
    if query.present?
      # 改进搜索，允许 join 查询书架信息（如果需要）
      # left_joins(current_slot: :bookshelf)
      # .where("books.title LIKE :q OR books.author LIKE :q OR books.isbn LIKE :q OR bookshelves.code LIKE :q", q: "%#{query}%")
      where("title LIKE :q OR author LIKE :q OR isbn LIKE :q", q: "%#{query}%")
    else
      all
    end
  end

  private

  def update_slot_occupancy
    # current_slot_id_before_last_save 是 Rails 提供的用于获取旧值的方法
    old_slot_id = current_slot_id_before_last_save
    new_slot_id = current_slot_id

    # 更新旧槽位状态
    Slot.find_by(id: old_slot_id)&.update(is_occupied: false)
    
    # 更新新槽位状态
    Slot.find_by(id: new_slot_id)&.update(is_occupied: true)
  end

  def current_slot_must_be_available
    return unless current_slot.present?
    # 检查目标 current_slot 是否已被其他 book 占用
    if current_slot.current_book.present? && current_slot.current_book != self
      errors.add(:current_slot_id, "已被书籍 '#{current_slot.current_book.title}' 占用")
    end
  end
  
  def intended_slot_must_be_available
     return unless intended_slot.present?
    # 检查目标 intended_slot 是否已是其他 book 的 intended_slot
    if intended_slot.intended_book.present? && intended_slot.intended_book != self
      errors.add(:intended_slot_id, "已被书籍 '#{intended_slot.intended_book.title}' 指定为目标位置")
    end
  end
end
