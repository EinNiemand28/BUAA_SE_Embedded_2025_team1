require "test_helper"

class Api::V1::SlotsControllerTest < ActionDispatch::IntegrationTest
  test "should get index" do
    get api_v1_slots_index_url
    assert_response :success
  end
end
