import consumer from "channels/consumer"

// 摄像头流接口，负责处理摄像头数据的接收和控制
const CameraStreamChannel = {
  subscription: null,
  isStreaming: false,
  
  connect: function() {
    if (this.subscription && this.subscription.consumer.connection.isOpen()) {
      console.log("[CameraStreamChannel.js] Already connected or connecting.");
      return this.subscription;
    }

    this.subscription = consumer.subscriptions.create("CameraStreamChannel", {
      // ActionCable 生命周期回调
      connected() {
        console.log("[CameraStreamChannel.js] Successfully connected to CameraStreamChannel on Rails.");
        document.dispatchEvent(new CustomEvent("camera-stream:connected"));
      },

      disconnected() {
        console.log("[CameraStreamChannel.js] Disconnected from CameraStreamChannel.");
        document.dispatchEvent(new CustomEvent("camera-stream:disconnected"));
      },

      // 接收来自Rails端的摄像头帧数据
      received(data) {
        console.log("[CameraStreamChannel.js] Received data from Rails:", {
          type: data.type,
          frame_id: data.frame_id,
          size: data.data_size,
          timestamp: data.timestamp
        });
        
        if (data.type === "camera_frame") {
          console.log("[CameraStreamChannel.js] Processing camera frame:", data.frame_id);
          // 分发摄像头帧事件
          document.dispatchEvent(new CustomEvent("camera-stream:frame", {
            detail: {
              frame_id: data.frame_id,
              timestamp: data.timestamp,
              format: data.format,
              width: data.width,
              height: data.height,
              data_url: data.data_url,
              data_size: data.data_size
            }
          }));
        } else if (data.status) {
          // 处理控制响应
          if (data.status === "success") {
            console.log("[CameraStreamChannel.js] Camera control success:", data);
            document.dispatchEvent(new CustomEvent("camera-stream:control_success", {
              detail: { message: data.message, enabled: data.enabled }
            }));
          } else if (data.status === "error") {
            console.log("[CameraStreamChannel.js] Camera control error:", data);
            document.dispatchEvent(new CustomEvent("camera-stream:control_error", {
              detail: { error: data.error }
            }));
          }
        } else {
          console.log("[CameraStreamChannel.js] Unknown message type:", data);
        }
      }
    });

    return this.subscription;
  },

  disconnect: function() {
    if (this.subscription) {
      this.subscription.unsubscribe();
      this.subscription = null;
      this.isStreaming = false;
      console.log("[CameraStreamChannel.js] Manually unsubscribed and disconnected.");
    }
  },

  // 切换摄像头流开关
  toggleStream: function(enable) {
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[CameraStreamChannel.js] Performing 'toggle_camera_stream': enable=${enable}`);
      return sub.perform('toggle_camera_stream', { enable: !!enable });
    }
    return Promise.reject("Subscription not available for toggle_camera_stream");
  },

  // 设置摄像头参数
  setCameraParams: function(params) {
    const sub = this.ensureConnected();
    if (sub) {
      console.log(`[CameraStreamChannel.js] Performing 'set_camera_params':`, params);
      return sub.perform('set_camera_params', { params: params });
    }
    return Promise.reject("Subscription not available for set_camera_params");
  },

  // 确保连接存在
  ensureConnected: function() {
    if (!this.subscription) {
      console.log("[CameraStreamChannel.js] No subscription found. Attempting to connect...");
      this.connect();
    }
    
    if (this.subscription && this.subscription.consumer.connection.isOpen()) {
      return this.subscription;
    } else {
      console.warn("[CameraStreamChannel.js] Connection not ready for camera stream operations.");
      return null;
    }
  },

  // 获取当前流状态
  getStreamingState: function() {
    return this.isStreaming;
  },

  // 设置流状态（由外部调用）
  setStreamingState: function(state) {
    this.isStreaming = !!state;
  }
};

export default CameraStreamChannel; 