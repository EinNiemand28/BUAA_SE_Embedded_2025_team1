class CreateTasks < ActiveRecord::Migration[7.2]
  def change
    create_table :tasks do |t|
      t.integer :task_type
      t.integer :status
      t.integer :priority
      t.references :book, null: true, foreign_key: true
      t.references :source_slot, null: true, foreign_key: { to_table: :slots }
      t.references :target_slot, null: true, foreign_key: { to_table: :slots }
      t.float :target_point_x
      t.float :target_point_y
      t.float :target_point_z
      t.float :target_orientation_z
      t.references :user, null: false, foreign_key: true
      t.references :parent_task, null: true, foreign_key: { to_table: :tasks }
      t.datetime :scheduled_at
      t.datetime :started_at
      t.datetime :completed_at
      t.json :progress_details
      t.json :result_data

      t.timestamps
    end

    add_index :tasks, :task_type
    add_index :tasks, :status
  end
end
