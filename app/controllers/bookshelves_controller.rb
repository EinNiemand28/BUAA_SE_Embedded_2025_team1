class BookshelvesController < ApplicationController
  before_action :set_bookshelf, only: [:show, :edit, :update, :destroy]
  before_action :authenticate # 假设管理书架都需要登录
  before_action :require_admin, except: [:index, :show] # 假设只有管理员可以增删改

  def index
    @transit_stations = Bookshelf.transit_stations.order(:code)
    @normal_shelves = Bookshelf.normal_shelves.order(:code)
  end

  def show
    # @books_on_shelf = @bookshelf.books.order(:title) # 后续实现 Slot 和 Book 的关联后添加
  end

  def new
    @bookshelf = Bookshelf.new
  end

  def create
    @bookshelf = Bookshelf.new(bookshelf_params)
    if @bookshelf.save
      redirect_to @bookshelf, notice: t('bookshelves.create.success')
    else
      render :new, status: :unprocessable_entity
    end
  end

  def edit
  end

  def update
    if @bookshelf.update(bookshelf_params)
      redirect_to @bookshelf, notice: t('bookshelves.update.success')
    else
      render :edit, status: :unprocessable_entity
    end
  end

  def destroy
    @bookshelf.destroy
    redirect_to bookshelves_url, notice: t('bookshelves.destroy.success'), status: :see_other
  end

  private

  def set_bookshelf
    @bookshelf = Bookshelf.find(params[:id])
  end

  def bookshelf_params
    params.require(:bookshelf).permit(
      :code, :name, :center_x, :center_y, :orientation, 
      :length, :width, :height, :levels, :slots_per_level, 
      :bottom_clearance, :level_height, :is_transit_station
    )
  end
end
