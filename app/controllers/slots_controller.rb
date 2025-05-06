class SlotsController < ApplicationController
  # API 控制器通常不需要 authenticity token，但需要认证
  # 如果你使用 token-based auth for API, 在这里处理
  # 如果 API 也使用 cookie session, 确保 ApplicationController 的 set_current_session 生效
  skip_before_action :verify_authenticity_token, raise: false # 禁用 CSRF for API
  before_action :set_bookshelf
  before_action :authenticate # 确保用户登录才能访问 API

  # GET /bookshelves/:bookshelf_id/slots
  def index
    slots = @bookshelf.slots.includes(:current_book).order(level: :asc, row: :asc)
    render json: slots.map { |slot| 
      {
        id: slot.id,
        level: slot.level,
        row: slot.row,
        is_occupied: slot.current_book.present?,
        current_book_id: slot.current_book&.id,
        current_book_title: slot.current_book&.title # 使用安全导航符避免nil错误
      }
    }
  end

  private

  def set_bookshelf
    @bookshelf = Bookshelf.find(params[:bookshelf_id])
  rescue ActiveRecord::RecordNotFound
    render json: { error: "Bookshelf not found" }, status: :not_found
  end
end
