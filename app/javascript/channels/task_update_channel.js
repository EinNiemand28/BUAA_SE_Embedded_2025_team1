// app/javascript/channels/task_update_channel.js
import consumer from "./consumer"

// 这个模块提供了一个函数，用于为特定的任务ID创建或获取一个ActionCable订阅。
// 它接收数据并通过自定义事件分发，具体的UI更新由监听这些事件的Stimulus控制器处理。

const taskSubscriptions = {}; // 用于缓存订阅对象，避免重复创建

function subscribeToTask(taskId) {
  if (!taskId) {
    console.error("[TaskUpdateChannel.js] Task ID is required to subscribe.");
    return null;
  }

  if (taskSubscriptions[taskId] && taskSubscriptions[taskId].consumer.connection.isOpen()) {
    // console.log(`[TaskUpdateChannel.js] Already subscribed to task ${taskId}.`);
    return taskSubscriptions[taskId];
  }

  console.log(`[TaskUpdateChannel.js] Creating subscription for task ${taskId}.`);
  const subscription = consumer.subscriptions.create(
    { channel: "TaskUpdateChannel", task_id: taskId }, // Rails端 TaskUpdateChannel 需要这个 task_id
    {
      connected() {
        console.log(`[TaskUpdateChannel.js] Successfully connected to updates for Task ${taskId}.`);
        document.dispatchEvent(new CustomEvent("task-update-channel:connected", { detail: { taskId: taskId } }));
      },

      disconnected() {
        console.warn(`[TaskUpdateChannel.js] Disconnected from updates for Task ${taskId}.`);
        document.dispatchEvent(new CustomEvent("task-update-channel:disconnected", { detail: { taskId: taskId } }));
        delete taskSubscriptions[taskId]; // 移除缓存，以便下次重新创建
      },

      rejected() {
        console.error(`[TaskUpdateChannel.js] Subscription rejected for Task ${taskId}.`);
        document.dispatchEvent(new CustomEvent("task-update-channel:rejected", { detail: { taskId: taskId } }));
        delete taskSubscriptions[taskId];
      },

      received(data) {
        // console.log(`[TaskUpdateChannel.js] Received data for Task ${taskId}:`, data);
        // 假设数据结构是 { type: "task_update", task: { ...task_payload... } } (由Task模型回调广播)
        if (data.type === "task_update" && data.task && data.task.id === taskId) {
          document.dispatchEvent(new CustomEvent("task:updated", { // 通用事件名
            detail: { task: data.task } // 将整个任务对象传递出去
          }));
        } else {
          // 可以处理其他类型的消息，如果TaskUpdateChannel也发送的话
        }
      }
    }
  );

  taskSubscriptions[taskId] = subscription;
  return subscription;
}

function unsubscribeFromTask(taskId) {
  if (taskSubscriptions[taskId]) {
    taskSubscriptions[taskId].unsubscribe();
    delete taskSubscriptions[taskId];
    console.log(`[TaskUpdateChannel.js] Unsubscribed from Task ${taskId}.`);
  }
}

// 导出订阅和取消订阅的函数，供Stimulus控制器按需调用
export { subscribeToTask, unsubscribeFromTask };

// 移除了 turbo:load 中的自动订阅逻辑。
// Stimulus 控制器应该负责在其 connect 时调用 subscribeToTask(this.taskIdValue)，
// 并在其 disconnect 时调用 unsubscribeFromTask(this.taskIdValue)。
// 你提供的版本中，task_update_channel.js 包含了大量的UI更新逻辑，这些逻辑更适合放在
// 对应的 Stimulus 控制器中（例如，一个 task_show_controller.js 或 mapping_monitor_controller.js）。
// 这里的版本更专注于 Channel 通信本身。