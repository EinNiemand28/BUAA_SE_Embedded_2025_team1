class CreateSlots < ActiveRecord::Migration[7.2]
  def change
    create_table :slots do |t|
      t.references :bookshelf, null: false, foreign_key: true
      t.integer :row, null: false
      t.integer :level, null: false
      t.float :relative_x, null: false
      t.float :relative_y, null: false
      t.float :relative_z, null: false
      t.boolean :is_occupied, default: false, null: false

      t.timestamps
    end

    add_index :slots, [ :bookshelf_id, :level, :row ], unique: true
  end
end
