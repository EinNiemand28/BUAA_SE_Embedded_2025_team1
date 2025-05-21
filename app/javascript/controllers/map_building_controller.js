// app/javascript/controllers/mapping_monitor_controller.js
import { Controller } from "@hotwired/stimulus"
import consumer from "channels/consumer" // 如果需要直接订阅
import { subscribeToTask, unsubscribeFromTask } from "channels/task_update_channel"

// 这个控制器专门用于监控一个特定建图任务的进度和预览。
// 它需要从 data-* attribute 获取 task_id。
// <div data-controller="mapping-monitor" data-mapping-monitor-task-id-value="123"> ... </div>

export default class extends Controller {
  static values = { taskId: Number }
  static targets = [ /* 根据你的监控UI定义targets, 如 statusText, progressBar, mapPreviewImage */ ]

  connect() {
    if (!this.taskIdValue) {
      console.warn("[MappingMonitorCtrl] No task ID provided. Cannot monitor.");
      this.element.innerHTML = "<p class='text-red-500'>错误：未提供任务ID以进行监控。</p>";
      return;
    }
    console.log(`[MappingMonitorCtrl] Monitoring Task ID: ${this.taskIdValue}`);
    this.taskSpecificSubscription = null;
    this.mapPreviewSubscription = null;

    this._subscribeToTaskUpdates(this.taskIdValue);
    this._subscribeToMapPreview(this.taskIdValue);
    document.addEventListener("task:updated", this._handleSpecificTaskUpdate.bind(this));
  }

  disconnect() {
    if (this.taskIdValue) {
      this._unsubscribeFromCurrentTaskAndPreview();
    }
    document.removeEventListener("task:updated", this._handleSpecificTaskUpdate.bind(this));
  }

  _subscribeToTaskUpdates(taskId) {
    if (this.taskSpecificSubscription) this.taskSpecificSubscription.unsubscribe();
    this.taskSpecificSubscription = subscribeToTask(taskId);
  }

  _subscribeToMapPreview(taskId) {
    if (this.mapPreviewSubscription) this.mapPreviewSubscription.unsubscribe();
    this.mapPreviewSubscription = consumer.subscriptions.create(
      { channel: "RobotFeedbackChannel", stream_identifier: `map_preview_task_${taskId}` },
      {
        connected: () => console.log(`[MappingMonitorCtrl] Subscribed to map preview for task ${taskId}.`),
        received: this._handleMapPreviewData.bind(this)
      }
    );
  }

  _unsubscribeFromCurrentTaskAndPreview() {
    if (this.taskIdValue) unsubscribeFromTask(this.taskIdValue);
    if (this.mapPreviewSubscription) this.mapPreviewSubscription.unsubscribe();
  }

  _handleSpecificTaskUpdate(event) {
    const taskData = event.detail.task;
    if (taskData && taskData.id === this.taskIdValue) {
      // console.log("[MappingMonitorCtrl] Task Update:", taskData);
      // 在这里更新此控制器管理的UI元素 (statusTextTarget, progressBarTarget 等)
      // ... (与 robot_controller.js 中 _handleSpecificTaskUpdate 类似但针对此组件的UI)
      if (this.hasStatusTextTarget) this.statusTextTarget.textContent = `任务状态: ${taskData.status}`;
      // ... 更新进度条等 ...

      if (["completed", "failed", "cancelled"].includes(taskData.status)) {
        if (this.hasStatusTextTarget) this.statusTextTarget.textContent += " - 任务已结束。";
        this._unsubscribeFromCurrentTaskAndPreview(); // 任务结束，停止监听
      }
    }
  }

  _handleMapPreviewData(data) {
    if (data.type === "map_preview_update" && data.task_id === this.taskIdValue) {
      // console.log("[MappingMonitorCtrl] Map Preview:", data);
      // 在这里更新此控制器管理的地图预览UI (mapPreviewImageTarget)
      // ... (与 robot_controller.js 中 _handleMapPreviewData 类似)
      if (this.hasMapPreviewImageTarget) {
        this.mapPreviewImageTarget.src = `data:image/png;base64,${data.map_image_data_base64}`;
      }
    }
  }
}