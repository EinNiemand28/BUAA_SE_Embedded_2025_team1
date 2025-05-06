class ChangeStatusTypeInBooks < ActiveRecord::Migration[7.1]
  def up
    # 首先创建一个新的临时整数列
    add_column :books, :status_int, :integer, default: 0, null: false
    
    # 更新临时列的值，基于原始状态的字符串
    Book.find_each do |book|
      case book.status
      when "在架", "available"
        book.update_column(:status_int, 0)
      when "借出", "borrowed"
        book.update_column(:status_int, 1)
      when "异常", "unavailable" 
        book.update_column(:status_int, 2)
      when "中转", "transit"
        book.update_column(:status_int, 3)
      else
        book.update_column(:status_int, 0) # 默认设置为available
      end
    end
    
    # 删除旧列
    remove_column :books, :status
    
    # 重命名新列为原来的名称
    rename_column :books, :status_int, :status
  end

  def down
    # 首先创建一个新的临时字符串列
    add_column :books, :status_str, :string, default: "在架", null: false
    
    # 更新临时列的值，基于整数状态
    Book.find_each do |book|
      case book.status
      when 0
        book.update_column(:status_str, "在架")
      when 1
        book.update_column(:status_str, "借出")
      when 2
        book.update_column(:status_str, "异常")
      when 3
        book.update_column(:status_str, "中转")
      else
        book.update_column(:status_str, "在架") # 默认设置为available
      end
    end
    
    # 删除旧列
    remove_column :books, :status
    
    # 重命名新列为原来的名称
    rename_column :books, :status_str, :status
  end
end 