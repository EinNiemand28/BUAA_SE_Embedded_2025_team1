class CreateRobotStatuses < ActiveRecord::Migration[7.2]
  def change
    create_table :robot_statuses do |t|
      t.integer :status, null: false, default: 0
      t.text :error_message
      t.boolean :is_emergency_stopped, default: false
      t.boolean :is_mapping, default: false
      t.boolean :is_navigating, default: false
      t.references :current_task, foreign_key: { to_table: :tasks }, null: true
      t.references :active_map, foreign_key: { to_table: :maps }, null: true

      t.timestamps
    end

    add_index :robot_statuses, :status
    add_index :robot_statuses, :is_emergency_stopped
  end
end
