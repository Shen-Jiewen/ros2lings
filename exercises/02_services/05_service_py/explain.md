# Python 服务（Python Service）— rclpy 中的服务通信

## 概念

Python 中的 ROS2 服务使用 `rclpy` 库，API 风格与 C++ 类似但更加 Pythonic。
回调函数需要返回 response 对象。

## C++ vs Python 服务对比

| 特性 | C++ | Python |
|------|-----|--------|
| 导入 | `#include <ros2lings_interfaces/srv/add_two_ints.hpp>` | `from ros2lings_interfaces.srv import AddTwoInts` |
| 创建服务 | `create_service<Type>(name, callback)` | `self.create_service(Type, name, callback)` |
| 创建客户端 | `create_client<Type>(name)` | `self.create_client(Type, name)` |
| 异步请求 | `client->async_send_request(req)` | `client.call_async(req)` |
| 等待结果 | `spin_until_future_complete(node, future)` | `rclpy.spin_until_future_complete(node, future)` |
| 回调返回 | 通过 response 指针写入 | 需要 `return response` |

## 服务服务器（Python）

```python
from ros2lings_interfaces.srv import AddTwoInts

class MyServer(Node):
    def __init__(self):
        super().__init__('my_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.handle_add)

    def handle_add(self, request, response):
        response.sum = request.a + request.b
        return response  # Python 必须返回 response！
```

## 服务客户端（Python）

```python
client = node.create_client(AddTwoInts, 'add_two_ints')
client.wait_for_service(timeout_sec=5.0)

request = AddTwoInts.Request()
request.a = 10
request.b = 20

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
result = future.result()
```

## 常见错误

### 1. 导入路径拼写错误
```python
# 错误
from example_interface.srv import AddTwoInts   # 少了 s

# 正确
from ros2lings_interfaces.srv import AddTwoInts
```

### 2. 回调缺少 self
```python
# 错误 — 类方法必须有 self
def handle_add(request, response):

# 正确
def handle_add(self, request, response):
```

### 3. 忘记返回 response
```python
# 错误 — Python 回调必须 return response
def handle_add(self, request, response):
    response.sum = request.a + request.b

# 正确
def handle_add(self, request, response):
    response.sum = request.a + request.b
    return response
```

## 关键点

- Python 服务回调必须有 `self` 参数（如果是类方法）
- Python 服务回调必须 `return response`（C++ 不需要）
- `call_async` 对应 C++ 的 `async_send_request`
- 包名区分大小写和复数形式，要仔细拼写
