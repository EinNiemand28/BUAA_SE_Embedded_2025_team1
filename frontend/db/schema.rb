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

ActiveRecord::Schema[7.2].define(version: 2025_06_09_114235) do
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
    t.integer "status", default: 0, null: false
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

  create_table "maps", force: :cascade do |t|
    t.string "name", null: false
    t.text "description"
    t.string "map_data_url"
    t.integer "created_by_user_id", null: false
    t.boolean "is_active", default: false
    t.datetime "created_at", null: false
    t.datetime "updated_at", null: false
    t.index ["created_by_user_id"], name: "index_maps_on_created_by_user_id"
    t.index ["is_active"], name: "index_maps_on_is_active"
    t.index ["name"], name: "index_maps_on_name"
  end

  create_table "robot_statuses", force: :cascade do |t|
    t.integer "status", default: 0, null: false
    t.text "error_message"
    t.boolean "is_emergency_stopped", default: false
    t.boolean "is_mapping", default: false
    t.boolean "is_navigating", default: false
    t.integer "current_task_id"
    t.integer "active_map_id"
    t.datetime "created_at", null: false
    t.datetime "updated_at", null: false
    t.index ["active_map_id"], name: "index_robot_statuses_on_active_map_id"
    t.index ["current_task_id"], name: "index_robot_statuses_on_current_task_id"
    t.index ["is_emergency_stopped"], name: "index_robot_statuses_on_is_emergency_stopped"
    t.index ["status"], name: "index_robot_statuses_on_status"
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

  create_table "system_logs", force: :cascade do |t|
    t.integer "log_type"
    t.integer "severity"
    t.text "message"
    t.string "source"
    t.integer "user_id"
    t.integer "task_id"
    t.integer "book_id"
    t.datetime "created_at", null: false
    t.datetime "updated_at", null: false
    t.index ["book_id"], name: "index_system_logs_on_book_id"
    t.index ["created_at", "log_type"], name: "index_system_logs_on_created_at_and_log_type"
    t.index ["created_at", "severity"], name: "index_system_logs_on_created_at_and_severity"
    t.index ["log_type"], name: "index_system_logs_on_log_type"
    t.index ["severity"], name: "index_system_logs_on_severity"
    t.index ["source"], name: "index_system_logs_on_source"
    t.index ["task_id"], name: "index_system_logs_on_task_id"
    t.index ["user_id"], name: "index_system_logs_on_user_id"
  end

  create_table "tasks", force: :cascade do |t|
    t.integer "task_type"
    t.integer "status"
    t.integer "priority"
    t.integer "book_id"
    t.integer "source_slot_id"
    t.integer "target_slot_id"
    t.float "target_point_x"
    t.float "target_point_y"
    t.float "target_point_z"
    t.float "target_orientation_z"
    t.integer "user_id", null: false
    t.integer "parent_task_id"
    t.datetime "scheduled_at"
    t.datetime "started_at"
    t.datetime "completed_at"
    t.json "progress_details"
    t.json "result_data"
    t.datetime "created_at", null: false
    t.datetime "updated_at", null: false
    t.integer "map_id"
    t.index ["book_id"], name: "index_tasks_on_book_id"
    t.index ["map_id"], name: "index_tasks_on_map_id"
    t.index ["parent_task_id"], name: "index_tasks_on_parent_task_id"
    t.index ["source_slot_id"], name: "index_tasks_on_source_slot_id"
    t.index ["status"], name: "index_tasks_on_status"
    t.index ["target_slot_id"], name: "index_tasks_on_target_slot_id"
    t.index ["task_type"], name: "index_tasks_on_task_type"
    t.index ["user_id"], name: "index_tasks_on_user_id"
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
  add_foreign_key "robot_statuses", "maps", column: "active_map_id"
  add_foreign_key "robot_statuses", "tasks", column: "current_task_id"
  add_foreign_key "sessions", "users"
  add_foreign_key "slots", "bookshelves"
  add_foreign_key "system_logs", "books"
  add_foreign_key "system_logs", "tasks"
  add_foreign_key "system_logs", "users"
  add_foreign_key "tasks", "books"
  add_foreign_key "tasks", "maps"
  add_foreign_key "tasks", "slots", column: "source_slot_id"
  add_foreign_key "tasks", "slots", column: "target_slot_id"
  add_foreign_key "tasks", "tasks", column: "parent_task_id"
  add_foreign_key "tasks", "users"
end
