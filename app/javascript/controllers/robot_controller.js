// app/javascript/controllers/robot_controller.js
import { Controller } from "@hotwired/stimulus"
import consumer from "channels/consumer"
import RobotTaskChannel from "channels/robot_task_channel"
import RobotControlChannel from "channels/robot_control_channel"
import RobotFeedbackInterface from "channels/robot_feedback_channel"
import { subscribeToTask, unsubscribeFromTask } from "channels/task_update_channel"

export default class extends Controller {
  static targets = [
    // 通用状态
    "statusIndicator", "statusText", "robotModelStatusText", "batteryBar", "batteryText",
    "positionX", "positionY", "linearVelocity", "angularVelocity", "connectionStatus",
    // 手动移动
    "speedSlider", "speedValue", "manualMoveControlsPanel", // 新增 manualMoveControlsPanel
    // 模式控制按钮
    "emergencyStopBtn", "resumeOperationBtn", "enableManualBtn", "disableManualBtn", // 新增
    // 建图
    "mapName", "mapDescription", "startMappingBtn", "cancelMappingBtn",
    "taskStatus", "progressBar", "progressText", "mapPreview", "mapPreviewPlaceholder",
    "mappingContainer"
  ]

  // 用于存储从data-attributes传递过来的初始机器人状态
  // Stimulus 会自动将 data-robot-initial-robot-status-value 转换为 initialRobotStatusValue
  static values = {
    initialMappingTaskId: Number,
    initialMappingTaskStatus: String,
    initialRobotStatus: String,       // e.g., "idle", "emergency_stopped"
    initialRobotIsEstopped: Boolean   // e.g., true, false
  }

  connect() {
    console.log("[RobotCtrl] connect() called.");
    this._updateSpeedDisplay();

    RobotTaskChannel.connect();
    RobotControlChannel.connect();
    RobotFeedbackInterface.connect();

    // 处理从data attribute传递的初始建图任务状态
    this.currentMappingTaskId = this.initialMappingTaskIdValue || null;
    const initialMappingStatus = this.initialMappingTaskStatusValue;

    this.taskSpecificSubscription = null;
    this.mapPreviewSubscription = null;

    this._addEventListeners();

    // 使用传递的初始机器人状态来设置UI
    this._updateModeControlButtonsVisibility(this.initialRobotStatusValue, this.initialRobotIsEstoppedValue);
    
    if (this.currentMappingTaskId && ["processing", "pending", "cancelling"].includes(initialMappingStatus)) {
      console.log(`[RobotCtrl] Resuming monitoring for mapping task ${this.currentMappingTaskId}`);
      this._updateUIForMappingTask(this.currentMappingTaskId, initialMappingStatus);
      this._subscribeToTaskUpdates(this.currentMappingTaskId);
      this._subscribeToMapPreview(this.currentMappingTaskId);
    } else {
      this._updateUIForNoTask();
    }
    this._updateConnectionStatusText();
  }

  disconnect() {
    console.log("[RobotCtrl] Disconnecting...");
    this._removeEventListeners();
    RobotTaskChannel.disconnect();
    RobotControlChannel.disconnect();
    RobotFeedbackInterface.disconnect();
    this._unsubscribeFromCurrentTaskAndPreview();
  }


  _addEventListeners() {
    // ... (之前的监听器保持不变) ...
    console.log("[RobotCtrl] Adding event listeners.");
    document.addEventListener("robot-feedback:connected", this._handleFeedbackConnected.bind(this));
    document.addEventListener("robot-feedback:disconnected", this._handleFeedbackDisconnected.bind(this));
    document.addEventListener("robot_status_model_update", this._handleRobotStatusModelUpdate.bind(this));
    document.addEventListener("robot_state_update", this._handleRobotSensorDataUpdate.bind(this));
    document.addEventListener("robot-task-channel:action_success", this._handleTaskActionSuccess.bind(this));
    document.addEventListener("robot-task-channel:action_error", this._handleTaskActionError.bind(this));
    document.addEventListener("task:updated", this._handleSpecificTaskUpdate.bind(this));
  }

  _removeEventListeners() {
    // ... (移除所有监听器) ...
    console.log("[RobotCtrl] Removing event listeners.");
    document.removeEventListener("robot-feedback:connected", this._handleFeedbackConnected.bind(this));
    document.removeEventListener("robot-feedback:disconnected", this._handleFeedbackDisconnected.bind(this));
    document.removeEventListener("robot_status_model_update", this._handleRobotStatusModelUpdate.bind(this));
    document.removeEventListener("robot_state_update", this._handleRobotSensorDataUpdate.bind(this));
    document.removeEventListener("robot-task-channel:action_success", this._handleTaskActionSuccess.bind(this));
    document.removeEventListener("robot-task-channel:action_error", this._handleTaskActionError.bind(this));
    document.removeEventListener("task:updated", this._handleSpecificTaskUpdate.bind(this));
  }

  _handleFeedbackConnected() {
    console.log("[RobotCtrl] _handleFeedbackConnected TRIGGERED");
    this.feedbackConnected = true;
    this._updateConnectionStatusText();
  }
  _handleFeedbackDisconnected() {
    console.log("[RobotCtrl] _handleFeedbackDisconnected TRIGGERED");
    this.feedbackConnected = false;
    this._updateConnectionStatusText();
  }

  _updateConnectionStatusText() {
    if (!this.hasConnectionStatusTarget) {
      console.warn("[RobotCtrl] connectionStatusTarget not found!");
      return;
    }
    // feedbackConnected 标志由 RobotFeedbackInterface 的连接状态回调设置
    if (this.feedbackConnected) {
      this.connectionStatusTarget.textContent = "实时反馈通道已连接";
      this.connectionStatusTarget.className = "text-xs font-medium text-green-600";
    } else {
      this.connectionStatusTarget.textContent = "实时反馈通道未连接";
      this.connectionStatusTarget.className = "text-xs font-medium text-red-600";
    }
  }

  _updateSpeedDisplay() {
    if (this.hasSpeedSliderTarget && this.hasSpeedValueTarget) {
      this.speedValue = parseFloat(this.speedSliderTarget.value);
      this.speedValueTarget.textContent = this.speedValue.toFixed(1);
    }
  }

  _handleRobotStatusModelUpdate(event) {
    const statusData = event.detail; // payload from RobotStatus model callback
    console.log("[RobotCtrl] _handleRobotStatusModelUpdate RECEIVED:", JSON.stringify(statusData));

    if (this.hasStatusTextTarget) this.statusTextTarget.textContent = statusData.status_text || statusData.status;
    if (this.hasRobotModelStatusTextTarget) this.robotModelStatusTextTarget.textContent = `模型状态: ${statusData.status_text || statusData.status}`;
    if (this.hasStatusIndicatorTarget) {
      const colorClass = this._getRobotStatusColor(statusData.status);
      this.statusIndicatorTarget.className = `h-2.5 w-2.5 rounded-full mr-2 flex-shrink-0 ${colorClass}`;
    }

    // 更新模式控制按钮的可见性和手动移动面板的可见性
    this._updateModeControlButtonsVisibility(statusData.status, statusData.is_emergency_stopped);

    // ... (之前处理建图任务与模型状态冲突的逻辑保持) ...
    if (statusData.status !== "mapping" && this.currentMappingTaskId && statusData.current_task_id !== this.currentMappingTaskId) {
      this._showNotification(`机器人模型状态已更新 (${statusData.status})，当前建图任务 #${this.currentMappingTaskId} 可能已结束或变更。`, "warning");
      this._updateUIForNoTask(); this._unsubscribeFromCurrentTaskAndPreview(); this.currentMappingTaskId = null;
    } else if (statusData.status === "mapping" && statusData.current_task_id && this.currentMappingTaskId !== statusData.current_task_id) {
      this.currentMappingTaskId = statusData.current_task_id;
      this._showNotification(`机器人开始新的建图任务 #${this.currentMappingTaskId}。`, "info");
      this._updateUIForMappingTask(this.currentMappingTaskId, "processing");
      this._subscribeToTaskUpdates(this.currentMappingTaskId); this._subscribeToMapPreview(this.currentMappingTaskId);
    }
  }

  _handleRobotSensorDataUpdate(event) {
    const sensorData = event.detail;
    // console.log("[RobotCtrl] _handleRobotSensorDataUpdate RECEIVED:", JSON.stringify(sensorData)); // 日志可能过于频繁

    if (sensorData.battery_level !== undefined) {
      if (this.hasBatteryTextTarget && this.hasBatteryBarTarget) {
        const level = parseFloat(sensorData.battery_level);
        this.batteryTextTarget.textContent = `${level.toFixed(1)}%`;
        this.batteryBarTarget.style.width = `${level}%`;
        // 更新电池条颜色
        this.batteryBarTarget.classList.remove('bg-green-500', 'bg-yellow-500', 'bg-red-500', 'bg-gray-300'); // 先移除所有颜色
        if (level > 70) this.batteryBarTarget.classList.add('bg-green-500');
        else if (level > 30) this.batteryBarTarget.classList.add('bg-yellow-500');
        else this.batteryBarTarget.classList.add('bg-red-500');
      }
    }
    // ... (位置和速度更新逻辑保持不变，确保访问 sensorData.pose 和 sensorData.velocity) ...
    const pose = sensorData.pose;
    if (pose) {
      if (pose.x !== undefined && this.hasPositionXTarget) this.positionXTarget.textContent = parseFloat(pose.x).toFixed(2);
      if (pose.y !== undefined && this.hasPositionYTarget) this.positionYTarget.textContent = parseFloat(pose.y).toFixed(2);
    }
    const velocity = sensorData.velocity;
    if (velocity) {
      if (velocity.linear !== undefined && this.hasLinearVelocityTarget) this.linearVelocityTarget.textContent = parseFloat(velocity.linear).toFixed(2);
      if (velocity.angular !== undefined && this.hasAngularVelocityTarget) this.angularVelocityTarget.textContent = parseFloat(velocity.angular).toFixed(2);
    }
  }

  // --- 模式控制按钮的显隐逻辑 ---
  _updateModeControlButtonsVisibility(currentStatusStr, isEstoppedBool) {
    console.log(`[RobotCtrl] Updating button visibility. Status: ${currentStatusStr}, Estopped: ${isEstoppedBool}`);

    const isIdle = currentStatusStr === "idle";
    const isManual = currentStatusStr === "manual_control";
    const isMapping = currentStatusStr === "mapping"; // 或其他任务状态
    // isEstoppedBool 来自 RobotStatus.is_emergency_stopped

    // 急停按钮 (通常总是可见，但其功能可能由后端权限控制)
    // if (this.hasEmergencyStopBtnTarget) this.emergencyStopBtnTarget.disabled = isEstoppedBool; // 如果急停了就不能再按

    // 从急停恢复到 Idle 按钮
    if (this.hasResumeOperationBtnTarget) {
      this.resumeOperationBtnTarget.classList.toggle("hidden", !isEstoppedBool);
    }
    // 启用手动模式按钮 (从急停状态)
    if (this.hasEnableManualBtnTarget) {
      this.enableManualBtnTarget.classList.toggle("hidden", !isEstoppedBool);
    }
    // 禁用手动模式按钮 (返回 Idle)
    if (this.hasDisableManualBtnTarget) {
      this.disableManualBtnTarget.classList.toggle("hidden", !isManual);
    }
    // 手动移动控制面板
    if (this.hasManualMoveControlsPanelTarget) {
      this.manualMoveControlsPanelTarget.classList.toggle("hidden", !isManual);
    }
    // 开始建图按钮 (只有在 Idle 状态且当前没有建图任务时才可用)
    if (this.hasStartMappingBtnTarget) {
      this.startMappingBtnTarget.disabled = !(isIdle && !this.currentMappingTaskId);
      this.startMappingBtnTarget.classList.toggle("opacity-50", !isIdle || !!this.currentMappingTaskId);
      this.startMappingBtnTarget.classList.toggle("cursor-not-allowed", !isIdle || !!this.currentMappingTaskId);
    }
  }

  // --- 模式控制 Action 方法 ---
  emergencyStop() {
    if (confirm("确定要触发紧急停止吗？机器人所有动作将立即停止。")) {
      console.log("[RobotCtrl] Performing 'emergency_stop'");
      RobotControlChannel.emergencyStop();
      // UI的即时反馈：例如禁用所有其他控制按钮，直到状态更新
      // this._updateModeControlButtonsVisibility("emergency_stopped", true); // 模拟本地快速更新
    }
  }

  resumeOperation() {
    console.log("[RobotCtrl] Performing 'resume_operation'");
    RobotControlChannel.performAction('resume_operation', {}); // 调用 RobotControlChannel.rb 中定义的方法
    // UI 等待后端状态更新
  }

  enableManualControl() {
    console.log("[RobotCtrl] Performing 'enable_manual_control'");
    RobotControlChannel.performAction('enable_manual_control', {});
  }

  disableManualControl() {
    console.log("[RobotCtrl] Performing 'disable_manual_control'");
    RobotControlChannel.performAction('disable_manual_control', {});
  }

  // ... (rest of _handleIsbnScan, startMapping, cancelMapping, _handleTaskActionSuccess, _handleTaskActionError as before) ...
  // ... (_handleSpecificTaskUpdate, _handleMapPreviewData, _handleTaskCompletion as before) ...
  // ... (_subscribeToTaskUpdates, _subscribeToMapPreview, _unsubscribeFromCurrentTaskAndPreview as before) ...
  // ... (_updateUIForMappingTask, _updateUIForNoTask, _getRobotStatusColor, _showNotification as before) ...
  // ... (move methods as before) ...

  // --- 建图任务相关方法 ---
  startMapping() {
    console.log("[RobotCtrl] startMapping called");
    if (!this.hasMapNameTarget || this.mapNameTarget.value.trim() === "") {
      this._showNotification("请输入地图名称。", "error");
      return;
    }
    if (this.currentMappingTaskId) {
      this._showNotification(`已有建图任务 #${this.currentMappingTaskId} 正在进行中。`, "warning");
      return;
    }

    const mapName = this.mapNameTarget.value.trim();
    const mapDescription = this.hasMapDescriptionTarget ? this.mapDescriptionTarget.value.trim() : "";

    this._updateUIForMappingTask(" new", "pending"); // UI上临时显示"New"，等待后端返回task_id

    RobotTaskChannel.createTask("MAP_BUILD_AUTO", { // 确保TaskType字符串与Rails Enum一致
      map_name: mapName,
      description: mapDescription
    });
    // 后续处理在 _handleTaskActionSuccess 中
  }

  cancelMapping() {
    console.log("[RobotCtrl] cancelMapping called");
    if (!this.currentMappingTaskId) {
      this._showNotification("没有正在进行的建图任务可取消。", "error");
      return;
    }
    if (confirm(`确定要取消建图任务 #${this.currentMappingTaskId} 吗？`)) {
      RobotTaskChannel.cancelTask(this.currentMappingTaskId);
    }
  }

  _handleTaskActionSuccess(event) {
    const { action, taskId, response } = event.detail;
    console.log(`[RobotCtrl] Task action '${action}' for task ID ${taskId} successful:`, response);

    if (action === "create_task" && response.task_type === "map_build_auto") { // 比较的是字符串
      this.currentMappingTaskId = taskId;
      this._showNotification(`建图任务 #${taskId} 已成功创建，状态: ${response.initial_status}`, "success");
      this._updateUIForMappingTask(taskId, response.initial_status);
      this._subscribeToTaskUpdates(taskId);
      this._subscribeToMapPreview(taskId);
    } else if (action === "cancel_task" && taskId === this.currentMappingTaskId) {
      this._showNotification(`任务 #${taskId} 的取消请求已发送。新状态: ${response.new_status}`, "info");
      if (this.hasCancelMappingBtnTarget) this.cancelMappingBtnTarget.disabled = true;
    }
  }

  _handleTaskActionError(event) {
    const { errors, response } = event.detail;
    console.error("[RobotCtrl] Task action failed:", errors, "Response:", response);
    this._showNotification(`操作失败: ${errors.join(', ')}`, "error");
    // 确保 response 和 response.action 存在
    if (response && response.action === "create_task") {
      this._updateUIForNoTask();
    }
  }

  _handleSpecificTaskUpdate(event) {
    const taskData = event.detail.task;
    if (!taskData || taskData.id !== this.currentMappingTaskId) return;

    console.log("[RobotCtrl] SpecificTaskUpdate for current mapping task:", JSON.stringify(taskData));

    if (this.hasTaskStatusTarget) {
      let statusMsg = `任务 #${taskData.id} (${taskData.type}): ${taskData.status}`;
      const progress = taskData.progress_details || {};
      const result = taskData.result_data || {};
      const lastRosMsg = progress.last_ros_message || result.ros_message;
      if (lastRosMsg) statusMsg += ` - ${lastRosMsg}`;
      this.taskStatusTarget.textContent = statusMsg;
    }

    if (this.hasProgressBarTarget && this.hasProgressTextTarget) {
      const progressPercent = (taskData.progress_details && taskData.progress_details.last_ros_progress !== undefined)
                       ? parseInt(taskData.progress_details.last_ros_progress)
                       : (taskData.status === "completed" ? 100 : (taskData.status === "pending" ? 0 : (this.progressBarTarget.style.width || "0%").replace('%','')));
      const safeProgress = Math.max(0, Math.min(100, progressPercent));
      this.progressBarTarget.style.width = `${safeProgress}%`;
      this.progressTextTarget.textContent = `${safeProgress}%`;
    }

    if (["completed", "failed", "cancelled"].includes(taskData.status)) {
      this._handleTaskCompletion(taskData);
    }
  }

  _handleMapPreviewData(data) {
    console.log("[RobotCtrl] _handleMapPreviewData RECEIVED:", data.type, data.task_id, "CurrentTask:", this.currentMappingTaskId);
    if (data.type === "map_preview_update" && data.task_id === this.currentMappingTaskId && this.hasMapPreviewTarget) {
      this.mapPreviewTarget.src = `data:image/png;base64,${data.map_image_data_base64}`;
      this.mapPreviewTarget.classList.remove("hidden");
      if (this.hasMapPreviewPlaceholderTarget) this.mapPreviewPlaceholderTarget.classList.add("hidden");
      console.log("[RobotCtrl] Map preview updated for task #", data.task_id);
    } else {
      console.warn("[RobotCtrl] Map preview data received, but conditions not met or target missing. Has Target?", this.hasMapPreviewTarget);
    }
  }

  _handleTaskCompletion(taskData) {
    console.log("[RobotCtrl] _handleTaskCompletion for task:", JSON.stringify(taskData));
    if (taskData.id !== this.currentMappingTaskId) return;

    let message = `建图任务 #${taskData.id} 已 ${taskData.status}.`;
    if (taskData.result_data?.final_ros_status) message += ` (ROS: ${taskData.result_data.final_ros_status})`;
    
    const notifType = (taskData.status === "completed") ? "success" : ((taskData.status === "failed" || taskData.status === "cancelled") ? "error" : "info");
    this._showNotification(message, notifType);

    if (taskData.status === "completed" && taskData.map_id) {
        this._showNotification(`地图 #${taskData.map_id} (${taskData.result_data?.map_name_in_rails || '新地图'}) 已创建。`, "success");
    }

    this._updateUIForNoTask();
    this._unsubscribeFromCurrentTaskAndPreview();
    this.currentMappingTaskId = null;
  }

  _subscribeToTaskUpdates(taskId) {
    if (!taskId) { console.warn("[RobotCtrl] Attempted to subscribe to task updates with no taskId."); return; }
    this._unsubscribeFromCurrentTaskAndPreview(); // 确保先取消旧的
    
    this.taskSpecificSubscription = subscribeToTask(taskId); // from task_update_channel.js
    console.log(`[RobotCtrl] Subscribed to task updates for #${taskId}.`);
  }

  _subscribeToMapPreview(taskId) {
    if (!taskId) { console.warn("[RobotCtrl] Attempted to subscribe to map preview with no taskId."); return; }
    // this.mapPreviewSubscription 已在 _unsubscribeFromCurrentTaskAndPreview 中处理
    
    this.mapPreviewSubscription = consumer.subscriptions.create(
      // Rails RobotFeedbackChannel#report_map_preview 会 broadcast到 "map_preview_task_TASKID"
      // JS Channel JS 订阅时，channel名是 Rails Channel 类名，附加参数用于区分流
      { channel: "RobotFeedbackChannel", stream_identifier: `map_preview_task_${taskId}` },
      {
        connected: () => console.log(`[RobotCtrl] Subscribed to map preview stream for task ${taskId}.`),
        disconnected: () => console.log(`[RobotCtrl] Disconnected from map preview stream for task ${taskId}.`),
        received: this._handleMapPreviewData.bind(this)
      }
    );
    console.log(`[RobotCtrl] Attempted subscription to map preview for #${taskId}.`);
  }
  
  _unsubscribeFromCurrentTaskAndPreview() {
    console.log(`[RobotCtrl] Unsubscribing from task ${this.currentMappingTaskId} and its preview.`);
    if (this.currentMappingTaskId && this.taskSpecificSubscription) {
      unsubscribeFromTask(this.currentMappingTaskId);
      this.taskSpecificSubscription = null;
    }
    if (this.mapPreviewSubscription) {
      this.mapPreviewSubscription.unsubscribe();
      this.mapPreviewSubscription = null;
    }
  }

  _updateUIForMappingTask(taskId, statusText = "未知") {
    console.log(`[RobotCtrl] Updating UI for mapping task #${taskId}, status: ${statusText}`);
    if (this.hasStartMappingBtnTarget) this.startMappingBtnTarget.disabled = true;
    if (this.hasCancelMappingBtnTarget) this.cancelMappingBtnTarget.disabled = false;
    if (this.hasTaskStatusTarget) this.taskStatusTarget.textContent = `任务 #${taskId} 状态: ${statusText}`;
    if (this.hasMapPreviewTarget) this.mapPreviewTarget.classList.add("hidden");
    if (this.hasMapPreviewPlaceholderTarget) this.mapPreviewPlaceholderTarget.classList.remove("hidden");
    if (this.hasProgressBarTarget) this.progressBarTarget.style.width = "0%";
    if (this.hasProgressTextTarget) this.progressTextTarget.textContent = "0%";
  }

  _updateUIForNoTask() {
    console.log("[RobotCtrl] Updating UI for no active mapping task.");
    if (this.hasStartMappingBtnTarget) this.startMappingBtnTarget.disabled = false;
    if (this.hasCancelMappingBtnTarget) this.cancelMappingBtnTarget.disabled = true;
    if (this.hasTaskStatusTarget) this.taskStatusTarget.textContent = "等待发起新的建图任务。";
    if (this.hasMapPreviewTarget) {
        this.mapPreviewTarget.classList.add("hidden");
        this.mapPreviewTarget.src = ""; // 清空预览图
    }
    if (this.hasMapPreviewPlaceholderTarget) this.mapPreviewPlaceholderTarget.classList.remove("hidden");
    if (this.hasProgressBarTarget) this.progressBarTarget.style.width = "0%";
    if (this.hasProgressTextTarget) this.progressTextTarget.textContent = "0%";
    if (this.hasMapNameTarget) this.mapNameTarget.value = "";
    if (this.hasMapDescriptionTarget) this.mapDescriptionTarget.value = "";
  }

  _getRobotStatusColor(statusEnumString) {
    const colorMap = {
      offline: 'bg-gray-400', idle: 'bg-green-400', mapping: 'bg-blue-400',
      navigating: 'bg-blue-400', fetching_book: 'bg-purple-400',
      returning_book: 'bg-purple-400', scanning: 'bg-indigo-400',
      error: 'bg-red-500', paused: 'bg-yellow-400', emergency_stopped: 'bg-red-600',
      processing: 'bg-blue-400', //  Task status
      pending: 'bg-yellow-400', // Task status
      completed: 'bg-green-400', // Task status
      failed: 'bg-red-500', // Task status
      cancelling: 'bg-orange-500',// Task status
      cancelled: 'bg-gray-400' // Task status
    };
    return colorMap[statusEnumString] || 'bg-gray-300';
  }

  _showNotification(message, type = "info") {
    if (window.showNotification) {
      window.showNotification(message, type);
    } else {
      const prefix = type.toUpperCase();
      alert(`[${prefix}] ${message}`);
    }
  }

  _canManuallyMove() {
    // 从UI或内部状态判断是否处于手动模式且非急停
    // 假设 this.robotModelStatusTextTarget.textContent 存储了类似 "模型状态: manual_control"
    // 或者依赖一个内部变量 this.currentRobotActualStatus
    const isManual = this.statusTextTarget.textContent.toLowerCase().includes("manual"); // 简单判断
    const isEstopped = this.initialRobotIsEstoppedValue; // 或者从 RobotStatusModelUpdate 更新一个内部变量

    if (!isManual) {
      this._showNotification("机器人未处于手动控制模式，无法执行移动操作。", "warning");
      return false;
    }
    if (isEstopped) {
      this._showNotification("机器人处于急停状态，无法执行移动操作。", "warning");
      return false;
    }
    return true;
  }

  moveForward()  { if (this._canManuallyMove() && RobotControlChannel.subscription) RobotControlChannel.move("forward",  parseFloat(this.hasSpeedSliderTarget ? this.speedSliderTarget.value : 0.5)); }
  moveBackward() { if (this._canManuallyMove() && RobotControlChannel.subscription) RobotControlChannel.move("backward", parseFloat(this.hasSpeedSliderTarget ? this.speedSliderTarget.value : 0.5)); }
  moveLeft()     { if (this._canManuallyMove() && RobotControlChannel.subscription) RobotControlChannel.move("left",     parseFloat(this.hasSpeedSliderTarget ? this.speedSliderTarget.value : 0.5)); }
  moveRight()    { if (this._canManuallyMove() && RobotControlChannel.subscription) RobotControlChannel.move("right",    parseFloat(this.hasSpeedSliderTarget ? this.speedSliderTarget.value : 0.5)); }
  stopMotion()   { if (this._canManuallyMove() && RobotControlChannel.subscription) RobotControlChannel.stopMotion(); }
}