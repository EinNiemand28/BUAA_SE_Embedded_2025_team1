# frontend

## 项目概述

- 目标：本项目是一个使用 Ruby on Rails 构建的 Web 应用，作为控制 ROS Noetic 机器人的前端界面，（预计）通过 ROS Bridge 与机器人通信，旨在提供一个用户友好的界面来展示机器人的状态和控制机器人

- 技术栈：Rails 7.2.2.1、Tailwind CSS、ROS Bridge

- 开发环境：WSL2 Ubuntu 22.04

## 开发计划

- 部署策略：先于本地开发，后期基于 MRSK + Docker 部署至云服务器

- 数据库：本地开发使用 SQLite，（预计）后期部署使用 PostgreSQL + Redis

```mermaid
graph LR
    A[Rails Server] -- WebSocket --> B[ROS Bridge]
    B -- Topics --> C[ROS System]
    C -- /odom --> B
    B -- Data --> A
    D[Browser] -- Action Cable --> A
    A -- Updates --> D
```

```mermaid
graph TD
    A[用户 Web 浏览器] -->|HTTP/HTTPS| B[Rails Web 应用]
    B -->|渲染页面| A
    A -->|WebSocket Action Cable| B
    B -->|WebSocket Action Cable| A

    B -->|WebSocket 连接到公网 IP| C[ROS 中间层节点]
    C -->|WebSocket 连接到公网 IP| B

    C -->|ROS Topics 订阅/发布| D[ROS 机器人节点]

    subgraph Internet / 公网
        B
    end

    subgraph 机器人机载电脑 / 本地网络
        C
        D
    end

    %% 交互说明
    A -- 浏览器发送命令 --> B
    B -- 通过 Action Cable 转发命令 --> C
    C -- 将命令发布到 ROS Topic --> D
    D -- 发布状态/传感器数据到 ROS Topic --> C
    C -- 通过 Action Cable 发送数据 --> B
    B -- 广播数据到连接的浏览器 --> A
```