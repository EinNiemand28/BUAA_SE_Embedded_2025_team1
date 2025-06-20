class AddSlotsToBooks < ActiveRecord::Migration[7.2]
  def change
    add_reference :books, :current_slot, null: true, foreign_key: { to_table: :slots }, index: false
    add_reference :books, :intended_slot, null: true, foreign_key: { to_table: :slots }, index: false

    add_index :books, :current_slot_id
    add_index :books, :intended_slot_id
  end
end
