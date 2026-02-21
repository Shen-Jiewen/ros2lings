# 服务（Service）— ROS2 的请求/应答通信机制

## 概念

Service 是 ROS2 中的同步请求/应答通信模式。与话题（Topic）的单向广播不同，
服务是一对一的：客户端发送请求，服务器处理请求并返回响应。

## 话题 vs 服务

| 特性 | Topic（话题） | Service（服务） |
|------|-------------|----------------|
| 模式 | 发布/订阅（广播） | 请求/应答（一对一） |
| 方向 | 单向 | 双向 |
| 频率 | 持续流式 | 按需调用 |
| 适用场景 | 传感器数据、状态 | 计算、查询、配置 |

## .srv 文件格式

```
# 请求部分
int64 a
int64 b
---
# 响应部分
int64 sum
```

用 `---` 分隔请求和响应两部分。

## 服务服务器创建流程

```
create_service<SrvType>("服务名", 回调函数)   ← 创建服务
        │
        ▼
回调函数(request, response) {                ← 处理请求
  response->字段 = 计算结果;                  ← 填写响应
}
```

## 回调函数签名

```cpp
void callback(
  const std::shared_ptr<SrvType::Request> request,
  std::shared_ptr<SrvType::Response> response)
{
  response->sum = request->a + request->b;
}
```

注意：request 是 `const` 的（只读），response 不是（需要写入）。

## 关键点

- 服务名称是字符串，不能有空格，推荐使用 `snake_case`
- 回调函数必须有两个参数：request 和 response
- 使用 `ros2lings_interfaces/srv/AddTwoInts` 等标准接口练习
- 服务服务器需要节点持续 `spin` 才能响应请求
