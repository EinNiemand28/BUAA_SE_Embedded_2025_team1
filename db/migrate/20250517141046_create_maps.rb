class CreateMaps < ActiveRecord::Migration[7.2]
  def change
    create_table :maps do |t|
      t.string :name, null: false
      t.text :description
      t.string :map_data_url
      t.integer :created_by_user_id, null: false
      t.boolean :is_active, default: false

      t.timestamps
    end

    add_index :maps, :name
    add_index :maps, :created_by_user_id
    add_index :maps, :is_active
  end
end
