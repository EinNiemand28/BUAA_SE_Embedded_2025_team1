// app/javascript/channels/robot_task_channel.js
import consumer from "./consumer"

// 这个模块导出一个对象，包含了与 RobotTaskChannel 交互的方法。
// Stimulus 控制器或其他JS代码可以导入并使用这个对象。

const RobotTaskChannel = {
  subscription: null, // 用于存储订阅对象

  connect: function() {
    if (this.subscription && this.subscription.consumer.connection.isOpen()) {
      console.log("[RobotTaskChannel.js] Already connected or connecting.");
      return this.subscription; // 返回现有订阅
    }

    this.subscription = consumer.subscriptions.create("RobotTaskChannel", {
      // ActionCable 生命周期回调
      connected() {
        console.log("[RobotTaskChannel.js] Successfully connected to RobotTaskChannel on Rails.");
        // 可以触发一个自定义事件，通知其他JS模块连接已成功
        document.dispatchEvent(new CustomEvent("robot-task-channel:connected"));
      },

      disconnected() {
        console.warn("[RobotTaskChannel.js] Disconnected from RobotTaskChannel on Rails.");
        document.dispatchEvent(new CustomEvent("robot-task-channel:disconnected"));
        // 注意：ActionCable consumer 会自动尝试重连，这里不需要手动处理
      },

      received(data) {
        // 这个回调处理从Rails RobotTaskChannel 主动推送 (broadcast) 或
        // perform 后由 transmit 回传的数据。
        console.log("[RobotTaskChannel.js] Received data:", data);

        if (data.status === "success" && data.task_id) {
          // 任务创建或取消请求成功
          document.dispatchEvent(new CustomEvent("robot-task-channel:action_success", {
            detail: {
              action: data.message && data.message.includes("cancellation") ? "cancel_task" : "create_task", // 简单判断
              taskId: data.task_id,
              response: data
            }
          }));
        } else if (data.status === "error") {
          // 任务创建或取消请求失败
          console.error("[RobotTaskChannel.js] Action failed:", data.errors);
          document.dispatchEvent(new CustomEvent("robot-task-channel:action_error", {
            detail: {
              errors: data.errors,
              response: data
            }
          }));
        }
        // 可以根据 data.type 或其他字段分发更具体的事件
      }
    });
    return this.subscription;
  },

  // 确保已连接，如果未连接则尝试连接
  ensureConnected: function() {
    if (!this.subscription || !this.subscription.consumer.connection.isOpen()) {
      console.log("[RobotTaskChannel.js] Not connected, attempting to connect...");
      return this.connect(); // connect 会返回 subscription 对象
    }
    return this.subscription;
  },

  // 方法：创建任务
  // taskType: 字符串, e.g., "MAP_BUILD_AUTO"
  // parameters: 对象, e.g., { map_name: "My Map", description: "..." }
  createTask: function(taskType, parameters = {}) {
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[RobotTaskChannel.js] Performing 'create_task': type=${taskType}, params=`, parameters);
      // perform 方法返回 Promise (如果服务器端 ActionCable 使用了 transmit_subscription_confirmation 或类似确认机制)
      // 或者它只是发送命令，结果通过 received 回调处理
      return sub.perform('create_task', {
        task_type: taskType,
        parameters: parameters
      });
    } else {
      console.error("[RobotTaskChannel.js] Cannot create task, subscription not available.");
      return Promise.reject("Subscription not available"); // 返回一个被拒绝的Promise
    }
  },

  // 方法：取消任务
  // taskId: 数字或字符串
  cancelTask: function(taskId) {
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[RobotTaskChannel.js] Performing 'cancel_task' for task ID: ${taskId}`);
      return sub.perform('cancel_task', { task_id: taskId });
    } else {
      console.error("[RobotTaskChannel.js] Cannot cancel task, subscription not available.");
      return Promise.reject("Subscription not available");
    }
  },

  disconnect: function() {
    if (this.subscription) {
      this.subscription.unsubscribe();
      this.subscription = null;
      console.log("[RobotTaskChannel.js] Manually unsubscribed and disconnected.");
    }
  }
};

// 首次加载时自动连接（可选，或者由使用它的控制器显式调用 connect）
// RobotTaskChannel.connect();

export default RobotTaskChannel;