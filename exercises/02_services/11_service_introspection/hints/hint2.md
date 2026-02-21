# 提示 2

答案提示:
- 列出服务: `ros2 service list` (不是 topic list 也不是 node list)
- Service 模式: `request_response` (客户端发送请求，服务端返回响应)
- 调用服务: `ros2 service call` (不是 send 也不是 invoke)
- 查看类型: `ros2 service type` (类似 `ros2 topic type`)
- 保证响应: `yes` (Service 是同步的，服务端正常运行时一定会响应)
