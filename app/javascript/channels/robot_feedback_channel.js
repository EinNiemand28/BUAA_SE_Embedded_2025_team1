// app/javascript/channels/robot_feedback_channel.js
import consumer from "./consumer"

// 这个JS Channel 订阅 Rails 的 RobotFeedbackChannel，接收来自ROS的各种反馈。
// 它将收到的不同类型的数据分发为更具体的自定义事件，供页面上的Stimulus控制器或其他JS监听。

const RobotFeedbackInterface = { // 改名为更通用的接口名称
  subscription: null,
  streamIdentifierForGeneralUpdates: "robot_general_updates_channel", // Rails端广播通用状态到这个流
  streamIdentifierForCamera: "robot_camera_stream_channel",         // Rails端广播摄像头数据到这个流
                                                                   // 地图预览和特定任务更新由各自的订阅处理

  connect: function() {
    if (this.subscription && this.subscription.consumer.connection.isOpen()) {
      console.log("[RobotFeedback.js] Already connected or connecting to general updates.");
      return; // 避免重复连接
    }

    // 订阅通用的机器人状态更新 (对应 RobotFeedbackChannel -> update_robot_state 的广播)
    this.subscription = consumer.subscriptions.create(
      // Rails端 RobotFeedbackChannel#update_robot_state 方法会广播到 "robot_general_updates_channel"
      // Rails端 RobotFeedbackChannel#report_isbn_scan 方法也会广播到 "robot_general_updates_channel"
      // Rails端 RobotStatus 模型的 after_commit 回调也会广播到 "robot_general_updates_channel"
      { channel: "RobotGeneralUpdatesChannel", stream_identifier: this.streamIdentifierForGeneralUpdates },
      {
        connected() {
          console.log(`[RobotFeedback.js] Successfully connected to RobotGeneralUpdatesChannel (stream: ${RobotFeedbackInterface.streamIdentifierForGeneralUpdates}).`);
          document.dispatchEvent(new CustomEvent("robot-feedback:connected"));
        },

        disconnected() {
          console.warn(`[RobotFeedback.js] Disconnected from RobotGeneralUpdatesChannel (stream: ${RobotFeedbackInterface.streamIdentifierForGeneralUpdates}).`);
          document.dispatchEvent(new CustomEvent("robot-feedback:disconnected"));
        },

        received(data) {
          // console.log("[RobotFeedback.js] Received data on general stream:", data);

          // 根据 data.type (由Rails端广播时设置) 分发事件
          // 这些事件名与 robot_controller.js 中监听的事件名对应
          if (data.type === "robot_state_update" && data.payload) {
            // 通用传感器状态 (位姿、速度、电量等)，非模型直接更新
            document.dispatchEvent(new CustomEvent("robot_state_update", { detail: data.payload }));
          } else if (data.type === "robot_status_model_update" && data.payload) {
            // RobotStatus 模型自身变化 (持久化状态)
            document.dispatchEvent(new CustomEvent("robot_status_model_update", { detail: data.payload }));
          } else if (data.type === "isbn_scan_update" && data.payload) {
            document.dispatchEvent(new CustomEvent("isbn_scan_update", { detail: data.payload }));
          }
          // 地图预览 (map_preview_update) 和特定任务更新 (task_update) 通过单独的订阅处理，不在这里接收。
        }
      }
    );

    // (可选) 如果摄像头流也通过 RobotFeedbackChannel 加不同 stream_identifier 广播
    // this.cameraSubscription = consumer.subscriptions.create(
    //   { channel: "RobotFeedbackChannel", stream_identifier: this.streamIdentifierForCamera },
    //   { /* ... connected, disconnected, received for camera frames ... */ }
    // );
  },

  disconnect: function() {
    if (this.subscription) {
      this.subscription.unsubscribe();
      this.subscription = null;
    }
    // if (this.cameraSubscription) this.cameraSubscription.unsubscribe();
    console.log("[RobotFeedback.js] Manually unsubscribed from feedback streams.");
  },

  // 注意：这个JS Channel主要是接收数据和分发事件。
  // Stimulus 控制器 (如 robot_controller.js) 会：
  // 1. 调用 RobotTaskChannel.js 和 RobotControlChannel.js 的方法来发送指令。
  // 2. 监听这个 RobotFeedbackInterface.js (通过document.addEventListener) 分发的事件来更新UI。
  // 3. 动态创建对 TaskUpdateChannel (特定任务ID) 和地图预览流的订阅。
};

// 页面加载时自动连接 (或由主控制器如 robot_controller.js 在其 connect 时调用)
// RobotFeedbackInterface.connect();

export default RobotFeedbackInterface;