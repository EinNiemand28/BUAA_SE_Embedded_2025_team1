require "test_helper"

class BookshelvesControllerTest < ActionDispatch::IntegrationTest
  test "should get index" do
    get bookshelves_index_url
    assert_response :success
  end

  test "should get show" do
    get bookshelves_show_url
    assert_response :success
  end

  test "should get new" do
    get bookshelves_new_url
    assert_response :success
  end

  test "should get create" do
    get bookshelves_create_url
    assert_response :success
  end

  test "should get edit" do
    get bookshelves_edit_url
    assert_response :success
  end

  test "should get update" do
    get bookshelves_update_url
    assert_response :success
  end

  test "should get destroy" do
    get bookshelves_destroy_url
    assert_response :success
  end
end
