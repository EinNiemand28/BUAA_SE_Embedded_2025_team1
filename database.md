

### 书籍表

| 属性名           | 中文名称         | 数据类型    | 备注                       |
| ---------------- | ---------------- | ----------- | -------------------------- |
| id               | 书籍ID           | INT         | 主键，自增                 |
| isbn             | 国际标准图书编号 | VARCHAR(25) |                            |
| title            | 书名             | VARCHAR(25) |                            |
| author           | 作者             | VARCHAR(25) |                            |
| publisher        | 出版社           | VARCHAR(35) |                            |
| publication_year | 出版年份         | YEAR        |                            |
| shelf_slot_id    | 书架插槽ID       | INT         | 外键，关联对应的书架插槽   |
| status           | 书籍状态         | ENUM        | 如“在架”、“借出”、“异常”等 |
| created_at       | 创建时间         | TIMESTAMP   | 默认当前时间               |
| updated_at       | 更新时间         | TIMESTAMP   | 每次修改记录自动更新       |



### 书架插槽表

| 属性名       | 中文名称     | 数据类型  | 备注                  |
| ------------ | ------------ | --------- | --------------------- |
| id           | 书架插槽ID   | INT       | 主键，自增            |
| bookshelf_id | 书架ID       | INT       | 外键，关联所在书架    |
| row          | 水平位置索引 | INT       | 从低到高              |
| level        | 垂直高度索引 | INT       | 从左到右              |
| relative_x   | X偏移        | INT       | 相对于书架中心的X偏移 |
| relative_y   | Y偏移        | INT       | 相对于书架中心的Y偏移 |
| relative_z   | 高度         | INT       | 相对于地面            |
| created_at   | 创建时间     | TIMESTAMP | 默认当前时间          |
| updated_at   | 更新时间     | TIMESTAMP | 每次修改记录自动更新  |



### 书架表

| 属性名           | 中文名称         | 数据类型    | 备注                                  |
| ---------------- | ---------------- | ----------- | ------------------------------------- |
| id               | 书架ID           | INT         | 主键，自增                            |
| code             | 书架编码         | VARCHAR(10) | 如 "A1"                               |
| name             | 描述性名称       | VARCHAR(20) |                                       |
| center_x         | X坐标            | DOUBLE      | 书架中心X坐标                         |
| center_y         | Y坐标            | DOUBLE      | 书架中心Y坐标                         |
| orientation      | 书架开口朝向角度 | DOUBLE      | (单位：弧度，0表示正东，pi/2表示正北) |
| length           | 书架长度         | DOUBLE      |                                       |
| width            | 书架宽度         | DOUBLE      |                                       |
| height           | 书架高度         | DOUBLE      |                                       |
| levels           | 书架层数         | INT         |                                       |
| slots_per_level  | 每层的插槽数量   | INT         |                                       |
| bottom_clearance | 最底层距地面高度 | DOUBLE      |                                       |
| level_height     | 每层高度         | DOUBLE      |                                       |
| created_at       | 创建时间         | TIMESTAMP   | 默认当前时间                          |
| updated_at       | 更新时间         | TIMESTAMP   | 每次修改记录自动更新                  |



### 用户表

| 属性名          | 中文名称 | 数据类型     | 备注                 |
| --------------- | -------- | ------------ | -------------------- |
| id              | 用户ID   | INT          | 主键，自增           |
| email           | 电子邮箱 | VARCHAR(50)  | 不可为空             |
| username        | 用户名   | VARCHAR(50)  | 不可为空             |
| password_digest | 密码     | VARCHAR(255) | 加密后的密码         |
| role            | 用户权限 | INT          | admin/user等         |
| created_at      | 创建时间 | TIMESTAMP    | 默认当前时间         |
| updated_at      | 更新时间 | TIMESTAMP    | 每次修改记录自动更新 |



### 会话表

| 属性名     | 中文名称   | 数据类型     | 备注                         |
| ---------- | ---------- | ------------ | ---------------------------- |
| id         | 会话ID     | INT          | 主键，自增                   |
| user_id    | 用户ID     | INT          | 外键，关联用户               |
| agent      | 用户代理   | VARCHAR(255) | 用户的浏览器或客户端标识信息 |
| ip_address | 登录IP地址 | VARCHAR(255) | 用户访问时的网络地址         |
| created_at | 注册时间   | TIMESTAMP    | 默认当前时间                 |
| updated_at | 更新时间   | TIMESTAMP    | 每次修改记录自动更新         |



### 任务表 (tasks)
| 属性名         | 中文名称     | 数据类型  | 备注                                             |
| -------------- | ------------ | --------- | ------------------------------------------------ |
| id             | 任务ID       | INT       | 主键，自增                                       |
| type           | 任务类型     | ENUM      | 'retrieve', 'return', 'scan', 'patrol'等         |
| status         | 任务状态     | ENUM      | 'pending', 'processing', 'completed', 'failed'等 |
| priority       | 优先级       | INT       | 数字越大优先级越高                               |
| book_id        | 书籍ID       | INT       | 外键，关联对应的书籍（可为空）                   |
| source_slot_id | 来源插槽ID   | INT       | 外键，关联对应的书架插槽（可为空）               |
| target_slot_id | 目标插槽ID   | INT       | 外键，关联对应的书架插槽（可为空）               |
| user_id        | 创建用户ID   | INT       | 外键，关联对应的用户                             |
| scheduled_at   | 计划执行时间 | DATETIME  | 任务计划开始的时间                               |
| started_at     | 实际开始时间 | DATETIME  | 任务实际开始的时间                               |
| completed_at   | 完成时间     | DATETIME  | 任务完成的时间                                   |
| description    | 任务描述     | TEXT      | 附加任务信息                                     |
| created_at     | 创建时间     | TIMESTAMP | 默认当前时间                                     |
| updated_at     | 更新时间     | TIMESTAMP | 每次修改记录自动更新                             |



### 系统日志表 (system_logs)
| 属性名         | 中文名称     | 数据类型     | 备注                                  |
| -------------- | ------------ | ------------ | ------------------------------------- |
| id             | 日志ID       | INT          | 主键，自增                            |
| type       | 日志类型     | ENUM         | 'system', 'robot', 'user', 'task', 'error' |
| severity       | 严重程度     | ENUM         | 'info', 'warning', 'error', 'critical' |
| message        | 日志消息     | TEXT         | 日志详细内容                          |
| source         | 日志来源     | VARCHAR(50)  | 产生日志的系统组件                    |
| user_id        | 用户ID       | INT          | 外键，关联相关用户（可为空）          |
| task_id        | 任务ID       | INT          | 外键，关联相关任务（可为空）          |
| book_id        | 书籍ID       | INT          | 外键，关联相关书籍（可为空）          |
| created_at     | 创建时间     | TIMESTAMP    | 日志产生时间                          |



### 机器人状态表 (robot_status)
| 属性名          | 中文名称   | 数据类型  | 备注                                    |
| --------------- | ---------- | --------- | --------------------------------------- |
| id              | 状态记录ID | INT       | 主键，自增                              |
| status          | 机器人状态 | ENUM      | 'idle', 'moving', 'grabbing', 'error'等 |
| battery_level   | 电池电量   | DOUBLE    | 百分比值 (0-100)                        |
| position_x      | X坐标      | DOUBLE    | 当前位置X坐标                           |
| position_y      | Y坐标      | DOUBLE    | 当前位置Y坐标                           |
| orientation     | 朝向角度   | DOUBLE    | 单位：弧度                              |
| current_task_id | 当前任务ID | INT       | 外键，关联当前正在执行的任务（可为空）  |
| error_message   | 错误信息   | TEXT      | 机器人错误状态描述（可为空）            |
| created_at      | 创建时间   | TIMESTAMP | 默认当前时间                            |
| updated_at      | 更新时间   | TIMESTAMP | 每次修改记录自动更新                    |

