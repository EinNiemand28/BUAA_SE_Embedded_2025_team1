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
  shelf.center_x = -5.0
  shelf.center_y = 2.0
  shelf.orientation = 0
  shelf.length = 0.9
  shelf.width = 0.3
  shelf.height = 1.2
  shelf.levels = 3
  shelf.slots_per_level = 1
  shelf.bottom_clearance = 0.06
  shelf.level_height = 0.38
  shelf.is_transit_station = true
end

if transit_shelf.persisted?
  puts "Transit bookshelf '#{transit_shelf.name}' created/found successfully."
else
  puts "Failed to create transit bookshelf: #{transit_shelf.errors.full_messages.join(", ")}"
end

puts "Creating sample normal bookshelves..."
bookshelf = Bookshelf.find_or_create_by!(code: "A-001") do |shelf|
  shelf.name = "普通书架 A-001"
  shelf.center_x = 5.0
  shelf.center_y = 2.0
  shelf.orientation = 3.14
  shelf.length = 0.9
  shelf.width = 0.3
  shelf.height = 1.2
  shelf.levels = 3
  shelf.slots_per_level = 1
  shelf.bottom_clearance = 0.06
  shelf.level_height = 0.38
  shelf.is_transit_station = false
end

if bookshelf.persisted?
  puts "Normal bookshelf '#{bookshelf.name}' created/found successfully."
else
  puts "Failed to create normal bookshelf: #{bookshelf.errors.full_messages.join(", ")}"
end

bookshelf = Bookshelf.find_or_create_by!(code: "A-002") do |shelf|
  shelf.name = "普通书架 A-002"
  shelf.center_x = 2.5
  shelf.center_y = 4.8
  shelf.orientation = -1.57
  shelf.length = 0.9
  shelf.width = 0.3
  shelf.height = 1.2
  shelf.levels = 3
  shelf.slots_per_level = 1
  shelf.bottom_clearance = 0.06
  shelf.level_height = 0.38
  shelf.is_transit_station = false
end

if bookshelf.persisted?
  puts "Normal bookshelf '#{bookshelf.name}' created/found successfully."
else
  puts "Failed to create normal bookshelf: #{bookshelf.errors.full_messages.join(", ")}"
end

bookshelf = Bookshelf.find_or_create_by!(code: "B-001") do |shelf|
  shelf.name = "普通书架 B-001"
  shelf.center_x = -2.5
  shelf.center_y = 2
  shelf.orientation = -1.57
  shelf.length = 0.9
  shelf.width = 0.3
  shelf.height = 1.2
  shelf.levels = 3
  shelf.slots_per_level = 1
  shelf.bottom_clearance = 0.06
  shelf.level_height = 0.38
  shelf.is_transit_station = false
end
if bookshelf.persisted?
  puts "Normal bookshelf '#{bookshelf.name}' created/found successfully."
else
  puts "Failed to create normal bookshelf: #{bookshelf.errors.full_messages.join(", ")}"
end
