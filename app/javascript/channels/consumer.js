// app/javascript/channels/consumer.js
import { createConsumer } from "@rails/actioncable"

// 辅助函数：从 HTML meta 标签获取内容
const getMetaValue = (name) => {
  const element = document.head.querySelector(`meta[name="${name}"]`)
  return element ? element.getAttribute("content") : null
}

// 构建 WebSocket URL
// 优先使用 meta 标签中定义的 URL (通常在 `app/views/layouts/application.html.erb` 中设置)
// 如果包含 current-user-token-id，则将其作为 user_token 参数附加到 URL，用于 Cable 连接认证
const createCableURL = () => {
  const cableUrlFromServer = getMetaValue("action-cable-url") // 例如：ws://localhost:3000/cable
  const userIdToken = getMetaValue("current-user-token-id") // Rails 端用 User#generates_token_for(:action_cable_connection) 生成

  let finalUrl = "/cable"; // 默认回退

  if (cableUrlFromServer) {
    finalUrl = cableUrlFromServer;
    console.log(`[ActionCable Consumer] Base URL from meta tag: ${finalUrl}`);

    if (userIdToken) {
      try {
        const urlObject = new URL(finalUrl); // 尝试解析，如果 finalUrl 是相对路径，需要提供 base
        urlObject.searchParams.append("user_token", userIdToken);
        finalUrl = urlObject.toString();
        console.log(`[ActionCable Consumer] User token found. Connecting with URL: ${finalUrl}`);
      } catch (e) { // 如果 finalUrl 是 "/cable" 这种相对路径，new URL 会失败
        if (finalUrl.includes("?")) {
          finalUrl += `&user_token=${encodeURIComponent(userIdToken)}`;
        } else {
          finalUrl += `?user_token=${encodeURIComponent(userIdToken)}`;
        }
        console.log(`[ActionCable Consumer] User token found (relative URL). Connecting with URL: ${finalUrl}`);
      }
    } else {
      console.log("[ActionCable Consumer] No user token found in meta tag. Connecting without user_token param.");
    }
  } else {
    console.warn("[ActionCable Consumer] 'action-cable-url' meta tag not found. Defaulting to '/cable'. Ensure it's set in your layout for proper WebSocket connection, especially in production.");
    // 如果需要，这里也可以尝试附加 token
    if (userIdToken) {
        finalUrl += `?user_token=${encodeURIComponent(userIdToken)}`;
        console.log(`[ActionCable Consumer] User token found (default URL). Connecting with URL: ${finalUrl}`);
    }
  }
  return finalUrl;
}

// 创建并导出 consumer 实例
// 其他JS文件可以导入这个 consumer 来创建订阅
export default createConsumer(createCableURL())