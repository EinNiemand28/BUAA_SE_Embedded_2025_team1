// app/javascript/channels/index.js

// 导入 consumer，确保它首先被加载和初始化
import consumer from "channels/consumer"

// 导入并初始化（如果它们有自启动逻辑）或导出各个 channel 模块
// 如果 channel 模块设计为按需连接 (例如通过Stimulus控制器调用 connect 方法)，
// 那么这里只需要确保它们被打包即可，不一定需要立即执行连接。

import RobotTaskChannel from "channels/robot_task_channel"
import RobotControlChannel from "channels/robot_control_channel"
import RobotFeedbackInterface from "channels/robot_feedback_channel" // 使用了新名称
import { subscribeToTask, unsubscribeFromTask } from "channels/task_update_channel" // 导入函数

// (可选) 立即连接某些总是需要的 channels
// RobotFeedbackInterface.connect(); // 例如，通用反馈可能总是需要

// 将 channel 对象或方法暴露给全局或其他模块（如果需要，但不推荐全局暴露）
// window.RobotTaskChannel = RobotTaskChannel;
// window.RobotControlChannel = RobotControlChannel;

console.log("[Channels Index] All channel modules loaded.");

// 确保其他JS可以通过导入来使用这些模块，例如：
// import RobotTaskChannel from 'channels/robot_task_channel';
// 在Stimulus控制器中。