# app/channels/task_update_channel.rb
class TaskUpdateChannel < ApplicationCable::Channel
  # 这个 Channel 用于前端订阅特定任务的更新。
  # 实际的广播逻辑在 Task 模型的 after_update_commit 回调中。

  def subscribed
    # 允许客户端订阅特定任务的更新流
    if params[:task_id].present?
      task_id = params[:task_id].to_i # 确保是整数
      # 可选：验证用户是否有权限订阅此任务的更新
      # task = Task.find_by(id: task_id)
      # if task && (task.user_id == connection.current_user&.id || connection.current_user&.admin?)
      stream_from "task_update_channel_#{task_id}"
      logger.info "[TaskUpdateChannel] User #{connection.current_user&.id || 'Client'} subscribed to updates for Task #{task_id}."
      # else
      #   logger.warn "[TaskUpdateChannel] Unauthorized attempt to subscribe to Task #{task_id} by User #{connection.current_user&.id}."
      #   reject
      # end
    else
      # 允许客户端订阅所有任务的简要更新流（如果需要）
      # stream_from "task_update_channel" # 全局流
      # logger.info "[TaskUpdateChannel] User #{connection.current_user&.id || 'Client'} subscribed to global task updates."
      # 通常，对于列表页面，轮询或Turbo Streams可能更合适，除非有强烈的实时需求
      reject # 强制要求 task_id，除非确实有全局订阅的需求
      logger.warn "[TaskUpdateChannel] Subscription rejected: task_id parameter is required."
    end
  end

  def unsubscribed
    # 停止所有来自此客户端的流
    stop_all_streams
    logger.info "[TaskUpdateChannel] User #{connection.current_user&.id || 'Client'} unsubscribed."
  end

  # （可选）如果前端需要主动请求某个任务的当前状态
  # def request_task_status(data)
  #   task_id = data['task_id']
  #   task = Task.find_by(id: task_id)
  #   if task && (task.user_id == connection.current_user&.id || connection.current_user&.admin?)
  #     # 直接通过 Task 模型的 broadcast_task_update 方法（如果它能被这样调用）
  #     # 或者手动构建 payload 并 transmit
  #     task_payload = { id: task.id, type: task.task_type, status: task.status, ... } # 完整payload
  #     transmit({ type: "task_update", task: task_payload })
  #   end
  # end
end
