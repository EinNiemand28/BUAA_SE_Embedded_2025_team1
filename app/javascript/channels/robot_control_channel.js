// app/javascript/channels/robot_control_channel.js
import consumer from "./consumer"

const RobotControlChannel = {
  subscription: null,

  connect: function() {
    if (this.subscription && this.subscription.consumer.connection.isOpen()) {
      console.log("[RobotControlChannel.js] Already connected or connecting.");
      return this.subscription;
    }

    this.subscription = consumer.subscriptions.create("RobotControlChannel", {
      connected() {
        console.log("[RobotControlChannel.js] Successfully connected to RobotControlChannel on Rails.");
        document.dispatchEvent(new CustomEvent("robot-control-channel:connected"));
      },

      disconnected() {
        console.warn("[RobotControlChannel.js] Disconnected from RobotControlChannel on Rails.");
        document.dispatchEvent(new CustomEvent("robot-control-channel:disconnected"));
      },

      received(data) {
        // 一般 RobotControlChannel 是单向的 (JS -> Rails -> ROS)
        // Rails 端通常不通过此 Channel broadcast 数据给JS，除非有特定确认消息
        console.log("[RobotControlChannel.js] Received data:", data);
        if (data.status === "error") { // 例如，如果Rails端校验失败并通过transmit返回错误
            console.error("[RobotControlChannel.js] Command error:", data.errors);
            document.dispatchEvent(new CustomEvent("robot-control-channel:command_error", { detail: data }));
        }
      }
    });
    return this.subscription;
  },

  ensureConnected: function() {
    if (!this.subscription || !this.subscription.consumer.connection.isOpen()) {
      return this.connect();
    }
    return this.subscription;
  },

  performAction: function(actionName, data = {}) {
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[RobotControlChannel.js] Performing '${actionName}' with data:`, data);
      return sub.perform(actionName, data);
    } else {
      console.error(`[RobotControlChannel.js] Cannot perform action '${actionName}', subscription not available.`);
      return Promise.reject(`Subscription not available for action ${actionName}`);
    }
  },

  move: function(direction, speed = 0.5) {
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[RobotControlChannel.js] Performing 'move': direction=${direction}, speed=${speed}`);
      return sub.perform('move', { direction: direction, speed: speed });
    }
    return Promise.reject("Subscription not available for move command");
  },

  stopMotion: function() {
    const sub = this.ensureConnected();
    if (sub) {
      console.log("[RobotControlChannel.js] Performing 'stop_motion'");
      return sub.perform('stop_motion', {}); // Rails端 action 名是 stop_motion
    }
    return Promise.reject("Subscription not available for stop_motion command");
  },

  cancelTask: function(taskId) {
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[RobotControlChannel.js] Performing 'cancel_task' for taskId=${taskId}`);
      return sub.perform('cancel_task', { task_id: taskId });
    }
    return Promise.reject("Subscription not available for cancel_task command");
  },

  completeMapBuild: function(mapName, description = "") {
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[RobotControlChannel.js] Performing 'complete_map_build': mapName=${mapName}, description=${description}`);
      return sub.perform('complete_map_build', { map_name: mapName, description: description });
    }
    return Promise.reject("Subscription not available for complete_map_build command");
  },

  emergencyStop: function() {
    const sub = this.ensureConnected();
    if (sub) {
      console.warn("[RobotControlChannel.js] Performing 'emergency_stop'");
      return sub.perform('emergency_stop', {});
    }
    return Promise.reject("Subscription not available for emergency_stop command");
  },

  toggleCameraStream: function(enable) { // enable 应该是布尔值
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[RobotControlChannel.js] Performing 'toggle_camera_stream': enable=${enable}`);
      return sub.perform('toggle_camera_stream', { enable: !!enable }); // 确保是布尔值
    }
    return Promise.reject("Subscription not available for toggle_camera_stream command");
  },

  disconnect: function() {
    if (this.subscription) {
      this.subscription.unsubscribe();
      this.subscription = null;
      console.log("[RobotControlChannel.js] Manually unsubscribed and disconnected.");
    }
  }
};

// RobotControlChannel.connect(); // 按需连接

export default RobotControlChannel;