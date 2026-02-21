# 提示 1

Python 类的实例方法第一个参数必须是 `self`。
看看 `listener_callback` 的参数列表，是不是少了什么？

```python
def listener_callback(self, msg):  # 注意 self
```
