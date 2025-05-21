class AddMapIdToTasks < ActiveRecord::Migration[7.2]
  def change
    add_reference :tasks, :map, null: true, foreign_key: true
  end
end
