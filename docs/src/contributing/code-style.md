# Code Style Guidelines

## Python (ROS 2 Packages)

- Follow **PEP 8** style guide
- Use **type hints** where appropriate
- Maximum line length: **120 characters**
- Use **docstrings** for classes and functions
- Import order: standard library, third-party, local imports

### Example

```python
#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node

class MyNode(Node):
    """Brief description of the node's purpose."""
    
    def __init__(self) -> None:
        super().__init__('my_node')
        self.get_logger().info("Node initialized")
```

## C++

- Follow **Google C++ Style Guide**
- Use **camelCase** for variables and functions
- Use **PascalCase** for classes and structs
- Maximum line length: **100 characters**
- Use meaningful variable names

### Example

```cpp
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    RCLCPP_INFO(this->get_logger(), "Node initialized");
  }

private:
  void processData();
};
```

## Documentation

- Update **README.md** for significant changes
- Add **docstrings/comments** for complex logic
- Update **package.xml** descriptions when needed
- Include **examples** in documentation