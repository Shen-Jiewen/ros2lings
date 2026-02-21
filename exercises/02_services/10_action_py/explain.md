# Python Action Server — 用 rclpy 创建 Action

## 概念

Python 中创建 Action Server 使用 `rclpy.action.ActionServer` 类。
与 C++ 版本类似，但 Python API 更简洁 — 只需要一个 `execute_callback`。

## 创建 Action Server

```python
from rclpy.action import ActionServer
from ros2lings_interfaces.action import Fibonacci

self._action_server = ActionServer(
    self,                    # Node 实例
    Fibonacci,               # Action 类型
    'fibonacci',             # Action 名称
    self.execute_callback    # 执行回调
)
```

注意参数顺序: `node, action_type, action_name, callback`

## execute_callback 结构

```python
def execute_callback(self, goal_handle):
    # 1. 获取目标
    order = goal_handle.request.order

    # 2. 计算并发布反馈
    feedback = Fibonacci.Feedback()
    goal_handle.publish_feedback(feedback)

    # 3. 标记成功
    goal_handle.succeed()

    # 4. 返回结果 (必须!)
    result = Fibonacci.Result()
    return result
```

## Python vs C++ 对比

| 特性 | Python | C++ |
|------|--------|-----|
| 回调数量 | 1 个 (execute_callback) | 3 个 (goal/cancel/accepted) |
| 反馈发布 | `goal_handle.publish_feedback()` | `goal_handle->publish_feedback()` |
| 成功标记 | `goal_handle.succeed()` | `goal_handle->succeed(result)` |
| 返回结果 | `return result` | 通过 `succeed(result)` 传入 |

## 关键点

- `ActionServer` 参数顺序: node, type, name, callback
- Fibonacci 用加法 (`+`)，不要写成乘法 (`*`)
- 反馈用 `publish_feedback()`，不是 `send_feedback()`
- `execute_callback` **必须** 返回 Result 对象
- Python Action Server 自动处理 goal 和 cancel
