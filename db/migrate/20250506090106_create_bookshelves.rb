class CreateBookshelves < ActiveRecord::Migration[7.2]
  def change
    create_table :bookshelves do |t|
      t.string :code, limit: 10, null: false
      t.string :name, limit: 50
      t.float :center_x, null: false
      t.float :center_y, null: false
      t.float :orientation
      t.float :length, null: false
      t.float :width, null: false
      t.float :height, null: false
      t.integer :levels, null: false
      t.integer :slots_per_level, null: false
      t.float :bottom_clearance
      t.float :level_height
      t.boolean :is_transit_station, default: false, null: false

      t.timestamps
    end

    add_index :bookshelves, :code, unique: true
    add_index :bookshelves, :name
  end
end
