require "sidekiq/web"

Rails.application.routes.draw do
  resources :system_logs, only: [ :index, :show ]
  resources :maps
  resources :tasks

  resources :bookshelves do
    resources :slots, only: [ :index ]
  end

  resources :books do
    collection do
      get :search
    end
  end

  # 语言切换路由
  get "switch_locale/:locale", to: "locales#switch", as: :switch_locale

  get  "sign_in", to: "sessions#new"
  post "sign_in", to: "sessions#create"
  get  "sign_up", to: "registrations#new"
  post "sign_up", to: "registrations#create"
  resources :sessions, only: [ :index, :show, :destroy ]
  resource  :password, only: [ :edit, :update ]

  # 添加个人资料路由
  resource :profile, only: [ :show ]

  # 更新用户管理路由，添加 edit, update, destroy
  resources :users, only: [ :index, :edit, :update, :destroy ]

  namespace :identity do
    resource :email,              only: [ :edit, :update ]
    resource :email_verification, only: [ :show, :create ]
  end

  # 系统日志路由
  resources :system_logs, only: [ :index, :show ]

  # 机器人管理路由
  resources :robots, only: [ :index, :show ] do
    collection do
      get "control"
      get "status"
      post "update_status"
      get "simulate_status"
      post "emergency_stop"
      post "resume"
      post "reset_status"
      post "set_active_map"
      post "start_work"
    end
  end

  # Action Cable Mount
  mount ActionCable.server => "/cable"

  # Sidekiq Web UI (仅对管理员开放)
  constraints lambda { |request|
      Current.user && Current.user.admin?
    } do
    mount Sidekiq::Web => "/sidekiq"
  end

  # Render dynamic PWA files from app/views/pwa/*
  get "service-worker" => "rails/pwa#service_worker", as: :pwa_service_worker
  get "manifest" => "rails/pwa#manifest", as: :pwa_manifest

  # Defines the root path route ("/")
  # root "posts#index"
  root "robots#index"
  # match '*path', via: :all, to: proc { [404, {}, ['Not Found']] }
end
