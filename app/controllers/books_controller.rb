class BooksController < ApplicationController
  before_action :set_book, only: [:show, :edit, :update, :destroy]
  before_action :authenticate, except: [:index, :show, :search]
  before_action :require_admin, except: [:index, :show, :search]
  
  def index
    @books = Book.all.order(created_at: :desc)
  end

  def show
  end

  def new
    @book = Book.new
  end

  def create
    @book = Book.new(book_params)
    
    if @book.save
      redirect_to @book, notice: t('books.create.success')
    else
      render :new, status: :unprocessable_entity
    end
  end

  def edit
  end

  def update
    if @book.update(book_params)
      respond_to do |format|
        format.html { redirect_to @book, notice: t('books.update.success') }
        format.json { render json: { success: true, message: t('books.update.success'), book: @book } }
      end
    else
      respond_to do |format|
        format.html { render :edit, status: :unprocessable_entity }
        format.json { render json: { success: false, errors: @book.errors.full_messages }, status: :unprocessable_entity }
      end
    end
  end

  def destroy
    @book.destroy
    redirect_to books_url, notice: t('books.destroy.success')
  end

  def search
    @query = params[:query]
    @books = Book.search(@query)
    render :index
  end
  
  private
  
  def set_book
    @book = Book.find(params[:id])
  end
  
  def book_params
    params.require(:book).permit(:isbn, :title, :author, :publisher, 
                                 :publication_year, :status, :cover_image,
                                 :current_slot_id, :intended_slot_id)
  end
end
