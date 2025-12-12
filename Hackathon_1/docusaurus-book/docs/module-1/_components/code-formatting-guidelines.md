# Code Example Formatting Guidelines

This document outlines the standard formatting and structure for code examples in the ROS 2 educational module.

## Python Code Examples

All Python code examples should follow this structure:

```python
# Brief description of what this code does
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialization code here
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Guidelines

1. **Import statements**: Always import rclpy and necessary components at the top
2. **Node class**: Use inheritance from `rclpy.node.Node`
3. **Constructor**: Initialize the parent class with a descriptive node name
4. **Main function**: Include proper initialization, spinning, and cleanup
5. **Comments**: Add comments to explain complex or non-obvious code sections
6. **Error handling**: Include try/except blocks for graceful shutdown
7. **Logging**: Use `self.get_logger().info()` for informative messages

## XML/URDF Examples

All URDF examples should follow this structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>

  <!-- Joints define connections between links -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Command Line Examples

For terminal commands, use the format:

```bash
# Description of what the command does
ros2 run package_name executable_name
```

## Inline Code Snippets

For short code snippets within text, use backticks: `ros2 topic list`

## Code File Organization

- Python examples: `examples/*.py`
- URDF examples: `examples/*.urdf`
- Launch files: `examples/*.py` or `examples/*.launch.py`
- Message definitions: `examples/*.msg` (if needed)

## Testing Code Examples

All code examples should be:
1. Complete and self-contained
2. Testable in a standard ROS 2 environment
3. Accompanied by expected output or behavior
4. Documented with any prerequisites or dependencies