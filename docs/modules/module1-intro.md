---
sidebar_position: 2
title: Module 1 - ROS 2 Foundations
---

# Module 1: ROS 2 Foundations

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is the second generation of the Robot Operating System. It's a flexible framework for writing robot software - a collection of tools and libraries that help you build robot applications.

## 📚 What is ROS 2?

ROS 2 is:
- **Middleware** - Enables communication between different software components
- **Distributed** - Runs across multiple machines
- **Open Source** - Developed collaboratively
- **Industry Standard** - Used in real-world robotics

## 🎯 Key Concepts

### **Nodes**
- Independent processes that perform computation
- Communicate with other nodes
- Run specific robot tasks

### **Topics**
- Named buses where nodes publish and subscribe to messages
- One-to-many communication pattern
- Asynchronous messaging

### **Services**
- Request/reply communication between nodes
- Synchronous communication
- For sporadic, non-continuous tasks

### **Actions**
- Long-running processes with feedback
- Used for goal-oriented tasks
- Support cancellation and feedback

## 🚀 Getting Started

### Installation
```bash
# For Ubuntu 22.04 (Jammy)
sudo apt update
sudo apt install ros-humble-desktop
```

### First Node
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 📝 Key Topics Covered

1. **Architecture** - Understanding DDS and graph concepts
2. **Installation** - Setup ROS 2 environment
3. **Nodes & Topics** - Basic communication
4. **Services** - Synchronous communication
5. **Actions** - Asynchronous goal-oriented tasks
6. **Packages** - Organizing code
7. **Build Systems** - Colcon and CMake

## ✅ Learning Outcomes

After this module, you'll understand:
- ✓ ROS 2 architecture and concepts
- ✓ How to create and run nodes
- ✓ Communication patterns (topics, services, actions)
- ✓ Package organization
- ✓ Debugging techniques

## 🔗 Resources

- [ROS 2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 GitHub Repository](https://github.com/ros2/ros2)

---

**Next:** Continue with ROS 2 installation and basic concepts
