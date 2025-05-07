# This file should ensure the existence of records required to run the application in every environment (production,
# development, test). The code here should be idempotent so that it can be executed at any point in every environment.
# The data can then be loaded with the bin/rails db:seed command (or created alongside the database with db:setup).
#
# Example:
#
#   ["Action", "Comedy", "Drama", "Horror"].each do |genre_name|
#     MovieGenre.find_or_create_by!(name: genre_name)
#   end

user = User.find_by(email: "1@1")

if user.present?
  user.update!(role: "admin")
else
  User.create!(
    username: "root", 
    email: "1@1", 
    password: "123123", 
    password_confirmation: "123123",
    role: "admin")
end

# 创建中转书架 (还书台/放书台)
puts "Creating transit bookshelf..."
transit_shelf = Bookshelf.find_or_create_by!(code: "TRANSIT-1") do |shelf|
  shelf.name = "中转还书台"
  shelf.center_x = 0.0
  shelf.center_y = -1.0 # 假设在机器人起始点附近
  shelf.orientation = 0.0 # 朝向正东
  shelf.length = 1.0 # 1米长
  shelf.width = 0.5  # 0.5米宽
  shelf.height = 0.8 # 0.8米高
  shelf.levels = 2     # 只有2层
  shelf.slots_per_level = 10 # 假设可以放5本书
  shelf.bottom_clearance = 0.1 # 底部离地10cm
  shelf.level_height = 0.7     # 层高70cm (总高度-底部间隙)
  shelf.is_transit_station = true
end

if transit_shelf.persisted?
  puts "Transit bookshelf '#{transit_shelf.name}' created/found successfully."
else
  puts "Failed to create transit bookshelf: #{transit_shelf.errors.full_messages.join(", ")}"
end

# 可以考虑创建一些普通书架作为示例
# puts "Creating sample normal bookshelves..."
# (1..3).each do |i|
#   shelf_code = "A-#{format('%03d', i)}" # 长度为 A-001 (5) 或 A-0001 (6)
#   bookshelf = Bookshelf.find_or_create_by!(code: shelf_code) do |shelf|
#     shelf.name = "普通书架 #{shelf_code}"
#     shelf.center_x = rand(1.0..5.0).round(2) # X坐标在1到5米之间
#     shelf.center_y = rand(-2.0..2.0).round(2) # Y坐标在-2到2米之间
#     shelf.orientation = [0, Math::PI/2, Math::PI, 3*Math::PI/2].sample # 随机朝向 (0, 90, 180, 270度)
#     shelf.length = 1.2
#     shelf.width = 0.3
#     shelf.height = 1.8
#     shelf.levels = 1
#     shelf.slots_per_level = 5
#     shelf.is_transit_station = false
#   end
#   if bookshelf.persisted?
#     puts "Sample bookshelf '#{bookshelf.name}' created/found successfully."
#   else
#     puts "Failed to create sample bookshelf #{shelf_code}: #{bookshelf.errors.full_messages.join(", ")}"
#   end
# end
