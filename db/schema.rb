# This file is auto-generated from the current state of the database. Instead
# of editing this file, please use the migrations feature of Active Record to
# incrementally modify your database, and then regenerate this schema definition.
#
# This file is the source Rails uses to define your schema when running `bin/rails
# db:schema:load`. When creating a new database, `bin/rails db:schema:load` tends to
# be faster and is potentially less error prone than running all of your
# migrations from scratch. Old migrations may fail to apply correctly if those
# migrations use external dependencies or application code.
#
# It's strongly recommended that you check this file into your version control system.

ActiveRecord::Schema[7.2].define(version: 2025_05_06_093308) do
  create_table "active_storage_attachments", force: :cascade do |t|
    t.string "name", null: false
    t.string "record_type", null: false
    t.bigint "record_id", null: false
    t.bigint "blob_id", null: false
    t.datetime "created_at", null: false
    t.index ["blob_id"], name: "index_active_storage_attachments_on_blob_id"
    t.index ["record_type", "record_id", "name", "blob_id"], name: "index_active_storage_attachments_uniqueness", unique: true
  end

  create_table "active_storage_blobs", force: :cascade do |t|
    t.string "key", null: false
    t.string "filename", null: false
    t.string "content_type"
    t.text "metadata"
    t.string "service_name", null: false
    t.bigint "byte_size", null: false
    t.string "checksum"
    t.datetime "created_at", null: false
    t.index ["key"], name: "index_active_storage_blobs_on_key", unique: true
  end

  create_table "active_storage_variant_records", force: :cascade do |t|
    t.bigint "blob_id", null: false
    t.string "variation_digest", null: false
    t.index ["blob_id", "variation_digest"], name: "index_active_storage_variant_records_uniqueness", unique: true
  end

  create_table "books", force: :cascade do |t|
    t.string "isbn", limit: 25, null: false
    t.string "title", limit: 255, null: false
    t.string "author", limit: 255, null: false
    t.string "publisher", limit: 255
    t.integer "publication_year"
    t.string "status", default: "在架", null: false
    t.datetime "created_at", null: false
    t.datetime "updated_at", null: false
    t.integer "current_slot_id"
    t.integer "intended_slot_id"
    t.index ["author"], name: "index_books_on_author"
    t.index ["current_slot_id"], name: "index_books_on_current_slot_id"
    t.index ["intended_slot_id"], name: "index_books_on_intended_slot_id"
    t.index ["isbn"], name: "index_books_on_isbn", unique: true
    t.index ["title"], name: "index_books_on_title"
  end

  create_table "bookshelves", force: :cascade do |t|
    t.string "code", limit: 10, null: false
    t.string "name", limit: 50
    t.float "center_x", null: false
    t.float "center_y", null: false
    t.float "orientation"
    t.float "length", null: false
    t.float "width", null: false
    t.float "height", null: false
    t.integer "levels", null: false
    t.integer "slots_per_level", null: false
    t.float "bottom_clearance"
    t.float "level_height"
    t.boolean "is_transit_station", default: false, null: false
    t.datetime "created_at", null: false
    t.datetime "updated_at", null: false
    t.index ["code"], name: "index_bookshelves_on_code", unique: true
    t.index ["name"], name: "index_bookshelves_on_name"
  end

  create_table "robot_commands", force: :cascade do |t|
    t.string "command_type", null: false
    t.text "parameters"
    t.integer "user_id"
    t.string "status", default: "pending"
    t.text "result"
    t.datetime "executed_at"
    t.datetime "created_at", null: false
    t.datetime "updated_at", null: false
    t.index ["command_type"], name: "index_robot_commands_on_command_type"
    t.index ["status"], name: "index_robot_commands_on_status"
    t.index ["user_id"], name: "index_robot_commands_on_user_id"
  end

  create_table "sessions", force: :cascade do |t|
    t.integer "user_id", null: false
    t.string "user_agent"
    t.string "ip_address"
    t.datetime "created_at", null: false
    t.datetime "updated_at", null: false
    t.index ["user_id"], name: "index_sessions_on_user_id"
  end

  create_table "slots", force: :cascade do |t|
    t.integer "bookshelf_id", null: false
    t.integer "row", null: false
    t.integer "level", null: false
    t.float "relative_x", null: false
    t.float "relative_y", null: false
    t.float "relative_z", null: false
    t.boolean "is_occupied", default: false, null: false
    t.datetime "created_at", null: false
    t.datetime "updated_at", null: false
    t.index ["bookshelf_id", "level", "row"], name: "index_slots_on_bookshelf_id_and_level_and_row", unique: true
    t.index ["bookshelf_id"], name: "index_slots_on_bookshelf_id"
  end

  create_table "users", force: :cascade do |t|
    t.string "email", limit: 50, null: false
    t.string "username", limit: 50, null: false
    t.string "password_digest", null: false
    t.integer "role", default: 0, null: false
    t.boolean "verified", default: false, null: false
    t.datetime "created_at", null: false
    t.datetime "updated_at", null: false
    t.index ["email"], name: "index_users_on_email", unique: true
    t.index ["username"], name: "index_users_on_username", unique: true
  end

  add_foreign_key "active_storage_attachments", "active_storage_blobs", column: "blob_id"
  add_foreign_key "active_storage_variant_records", "active_storage_blobs", column: "blob_id"
  add_foreign_key "books", "slots", column: "current_slot_id"
  add_foreign_key "books", "slots", column: "intended_slot_id"
  add_foreign_key "robot_commands", "users"
  add_foreign_key "sessions", "users"
  add_foreign_key "slots", "bookshelves"
end
