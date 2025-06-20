require "test_helper"

class SystemLogsControllerTest < ActionDispatch::IntegrationTest
  test "should get index" do
    get system_logs_index_url
    assert_response :success
  end

  test "should get show" do
    get system_logs_show_url
    assert_response :success
  end
end
