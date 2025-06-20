class CreateSystemLogs < ActiveRecord::Migration[7.2]
  def change
    create_table :system_logs do |t|
      t.integer :log_type
      t.integer :severity
      t.text :message
      t.string :source
      t.references :user, null: true, foreign_key: true
      t.references :task, null: true, foreign_key: true
      t.references :book, null: true, foreign_key: true

      t.timestamps
    end
    
    add_index :system_logs, :log_type
    add_index :system_logs, :severity
    add_index :system_logs, :source
  end
end
