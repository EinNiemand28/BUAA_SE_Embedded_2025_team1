class CreateBooks < ActiveRecord::Migration[7.2]
  def change
    create_table :books do |t|
      t.string :isbn, limit: 25, null: false
      t.string :title, limit: 255, null: false
      t.string :author, limit: 255, null: false
      t.string :publisher, limit: 255
      t.integer :publication_year
      t.integer :status, default: 0, null: false

      t.timestamps
    end

    add_index :books, :isbn, unique: true
    add_index :books, :title
    add_index :books, :author
  end
end
