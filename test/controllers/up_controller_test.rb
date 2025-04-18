require "test_helper"

class UpControllerTest < ActionDispatch::IntegrationTest
  test "should get show" do
    get up_show_url
    assert_response :success
  end
end
