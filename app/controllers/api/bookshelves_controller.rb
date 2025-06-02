# app/controllers/api/bookshelves_controller.rb
module Api
  class BookshelvesController < ApplicationController
    before_action :require_user_session
    before_action :set_bookshelf, only: [:show]

    def show
      render json: {
        bookshelf: {
          id: @bookshelf.id,
          code: @bookshelf.code,
          name: @bookshelf.name,
          center_x: @bookshelf.center_x,
          center_y: @bookshelf.center_y,
          width: @bookshelf.width,
          orientation: @bookshelf.orientation || 0.0
        }
      }
    end

    private

    def set_bookshelf
      @bookshelf = Bookshelf.find(params[:id])
    end
  end
end 