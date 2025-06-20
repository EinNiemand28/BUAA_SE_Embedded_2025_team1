// app/javascript/channels/robot_control_channel.js
import consumer from "channels/consumer"

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
        console.log("[RobotControlChannel.js] Disconnected from RobotControlChannel.");
        document.dispatchEvent(new CustomEvent("robot-control-channel:disconnected"));
      },

      received(data) {
        console.log("[RobotControlChannel.js] Received data from Rails:", data);
        
        if (data.status === "success") {
          document.dispatchEvent(new CustomEvent("robot-control-channel:command_success", {
            detail: { data: data }
          }));
        } else if (data.status === "error") {
          document.dispatchEvent(new CustomEvent("robot-control-channel:command_error", {
            detail: { 
              command: data.command || "unknown",
              errors: data.errors || [data.error || "未知错误"]
            }
          }));
        }
      }
    });

    return this.subscription;
  },

  // --- 移动控制方法 ---
  move: function(direction, speed) {
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
      return sub.perform('stop_motion', {});
    }
    return Promise.reject("Subscription not available for stop_motion command");
  },

  // --- 系统控制方法 ---
  emergencyStop: function() {
    const sub = this.ensureConnected();
    if (sub) {
      console.log("[RobotControlChannel.js] Performing 'emergency_stop'");
      return sub.perform('emergency_stop', {});
    }
    return Promise.reject("Subscription not available for emergency_stop command");
  },

  resumeOperation: function() {
    const sub = this.ensureConnected();
    if (sub) {
      console.log("[RobotControlChannel.js] Performing 'resume_operation'");
      return sub.perform('resume_operation', {});
    }
    return Promise.reject("Subscription not available for resume_operation command");
  },

  // --- 手动控制模式 ---
  enableManualControl: function() {
    const sub = this.ensureConnected();
    if (sub) {
      console.log("[RobotControlChannel.js] Performing 'enable_manual_control'");
      return sub.perform('enable_manual_control', {});
    }
    return Promise.reject("Subscription not available for enable_manual_control command");
  },

  disableManualControl: function() {
    const sub = this.ensureConnected();
    if (sub) {
      console.log("[RobotControlChannel.js] Performing 'disable_manual_control'");
      return sub.perform('disable_manual_control', {});
    }
    return Promise.reject("Subscription not available for disable_manual_control command");
  },

  // --- 建图控制 ---
  completeMapBuild: function(mapName, mapDescription) {
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[RobotControlChannel.js] Performing 'complete_map_build': name=${mapName}`);
      return sub.perform('complete_map_build', { 
        map_name: mapName, 
        map_description: mapDescription 
      });
    }
    return Promise.reject("Subscription not available for complete_map_build command");
  },

  // --- 摄像头控制 ---
  toggleCameraStream: function(enable) {
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[RobotControlChannel.js] Performing 'toggle_camera_stream': enable=${enable}`);
      return sub.perform('toggle_camera_stream', { enable: !!enable });
    }
    return Promise.reject("Subscription not available for toggle_camera_stream command");
  },

  // --- 通用移动指令发送方法 ---
  sendMovementCommand: function(command) {
    // command应包含 linear_velocity 和 angular_velocity 字段
    let direction = "stop";
    let speed = 0;
    
    if (command.linear_velocity > 0) {
      direction = "forward";
      speed = Math.abs(command.linear_velocity);
    } else if (command.linear_velocity < 0) {
      direction = "backward"; 
      speed = Math.abs(command.linear_velocity);
    } else if (command.angular_velocity > 0) {
      direction = "left";
      speed = Math.abs(command.angular_velocity);
    } else if (command.angular_velocity < 0) {
      direction = "right";
      speed = Math.abs(command.angular_velocity);
    }
    
    return this.move(direction, speed);
  },

  // --- 辅助方法 ---
  ensureConnected: function() {
    if (!this.subscription) {
      console.log("[RobotControlChannel.js] No subscription found. Attempting to connect...");
      this.connect();
    }
    
    if (this.subscription && this.subscription.consumer.connection.isOpen()) {
      return this.subscription;
    } else {
      console.warn("[RobotControlChannel.js] Connection not ready for robot control operations.");
      return null;
    }
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