# 图书馆机器人课程项目

本项目是北京航空航天大学《软件工程》课程的课程实践成果，旨在设计并实现一个基于 ROS 的图书馆智能机器人系统，支持从 Web 前端下发任务、控制机器人导航与取书，并提供完整的系统文档与开发实现。

项目由三部分组成：

- `document/`：课程项目的开发文档，包括需求说明、设计文档、测试报告等
- `frontend/`：基于 Ruby on Rails 构建的 Web 应用，提供图形界面用于管理书籍、书架及控制机器人
- `ros_end/`：基于 ROS Noetic 的机器人后端，包含仿真与实际部署功能包，用于地图构建、导航、任务执行等



## 📁 项目结构

```bash
project/
├── document/      # 项目开发文档
├── frontend/      # Web 前端（Rails）
├── ros_end/       # ROS 后端控制系统
└── demo.mp4       # 系统展示视频
```


## 📍 功能概览

### 🌐 前端（frontend/）

- 使用 Rails + Tailwind CSS + Stimulus 构建
- 实现功能包括：
  - 图书、书架、库位等管理界面
  - 用户权限系统与认证机制
  - 与机器人进行双向通信的任务调度面板
  - 状态仪表盘与 WebSocket 实时通信
- 可基于 Docker 部署于本地或云服务器，支持 PostgreSQL 与 Redis

详见 frontend/ 目录下的 [README](https://github.com/EinNiemand28/BUAA_SE_Embedded_2025_team1/blob/main/frontend/README.md) 及 [docs/](https://github.com/EinNiemand28/BUAA_SE_Embedded_2025_team1/tree/main/frontend/docs) 前端文档。



### 🤖 ROS 后端（ros_end/）

- 使用 ROS Noetic 框架，支持仿真（Gazebo）与实机测试
- 功能包包括：
  - `library_robot_interfaces`: ROS 中间层接口，负责前端与 ROS 系统的通信
  - `gazebosim_demo`: 仿真环境配置与演示
  - `realbot_demo`: 实际机器人驱动接口
- 支持对 Web 端下发的任务进行调度，执行导航、取书、地图管理等功能
- 提供标准 ROS 启动、构建与运行流程

详见 ros_end/ 目录下的 [README](https://github.com/EinNiemand28/BUAA_SE_Embedded_2025_team1/blob/main/ros_end/README.md)。



### 📄 项目文档（document/）

- 软件开发计划（SDP）
- 软件需求规格说明（SRS）
- 软件设计说明（SDD）
- 软件测试计划与报告（STP / STR）
- 项目总结文档

详见 document/ 目录下各文档。


## 运行与部署说明

参考各子目录下的 README 文件，其中包含详细的运行环境配置、依赖安装、构建与运行步骤。


## 项目完成状态
- ✅ 基本功能已实现：ROS 与 Web 应用的集成、实时通信、任务调度等
- ✅ 文档撰写完整，符合课程软件工程实践流程
- 🚧 部分功能仍待优化：书架巡检等复杂指令、高精度地图构建等


## 项目成员（Team 1）
- [EinNiemand28](https://github.com/EinNiemand28)
- [GreatMitchell](https://github.com/GreatMitchell)
- [fuhang23](https://github.com/fuhang23)


>本项目仅用于展示课程学习成果，部分代码与功能可能不完整或存在待优化之处。欢迎提出建议与反馈！
