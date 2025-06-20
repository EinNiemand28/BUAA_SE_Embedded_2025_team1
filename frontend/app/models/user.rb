# app/models/user.rb
class User < ApplicationRecord
  has_secure_password

  # Token generation for email verification and password reset
  generates_token_for :email_verification, expires_in: 2.days do
    email
  end
  generates_token_for :password_reset, expires_in: 20.minutes do
    password_salt.last(10) # Or any other unique, non-sensitive data
  end
  # 新增: 为 ActionCable 连接生成 token (如果通过 token 验证)
  generates_token_for :action_cable_connection, expires_in: 1.day do
    id # 使用用户ID作为 token 的一部分，确保唯一性
  end


  # 关联关系
  has_many :sessions, dependent: :destroy
  has_many :tasks, dependent: :destroy # 如果用户被删除，其创建的任务也应被删除或妥善处理
  has_many :created_maps, class_name: "Map", foreign_key: "created_by_user_id", dependent: :nullify # 如果用户被删除，其创建的地图将created_by_user_id设为null

  # 验证
  validates :email, presence: { message: "邮箱不能为空" },
                    uniqueness: { case_sensitive: false, message: "邮箱已被注册" },
                    format: { with: URI::MailTo::EMAIL_REGEXP, message: "邮箱格式不正确" }
  validates :username, presence: { message: "用户名不能为空" },
                       uniqueness: { case_sensitive: false, message: "用户名已被使用" },
                       length: { minimum: 3, maximum: 50, message: "用户名长度应为3-50个字符" }
  # password验证由has_secure_password处理，允许在更新时为空（不修改密码）
  validates :password, length: { minimum: 6, message: "密码长度至少为6位" }, if: -> { password.present? || new_record? }


  # 枚举：用户角色
  enum :role, [
    :user,  # 普通用户
    :admin  # 管理员
  ]

  # 规范化输入
  normalizes :email, with: ->(email) { email.strip.downcase }
  normalizes :username, with: ->(username) { username.strip }

  # 回调
  # 如果邮箱更改，则将verified状态设为false，需要重新验证
  before_validation if: :email_changed?, on: :update do
    self.verified = false
  end

  # 如果密码更改，则删除该用户除当前会话外的所有其他会
  after_update if: :password_digest_previously_changed? do
    sessions.where.not(id: Current.session).delete_all if Current.session # 确保 Current.session 存在
  end

  # 实例方法

  # 判断用户是否已验证邮箱
  def verified_email?
    verified?
  end
end
