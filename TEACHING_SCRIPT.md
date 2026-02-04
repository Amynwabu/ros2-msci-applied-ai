# ROS2 Comprehensive Teaching Script
## High-Engagement Video Lessons

---

# LESSON 02: ROS2 Packages and Nodes

## OPENING
Ever wondered how robots organize code? In ROS2, we use Packages (containers) and Nodes (specialized processes). By the end, you'll build ROS2 nodes from scratch!

## SECTION 1: What is a ROS2 Package?

A **Package** is a container for:
- Source code
- Metadata
- Dependencies
- Configuration

**Why Important**:
1. Reusability - Share with others
2. Organization - Everything has a place
3. Scalability - Projects grow without chaos
4. Collaboration - Teams find things easily

## SECTION 2: Restaurant Analogy

**Package** = Restaurant
**Node** = Staff member (waiter, chef)
**Graph** = How they communicate

Why this works:
- Waiter crashes? Chef keeps cooking
- Waiter doesn't know HOW chef cooks
- You can have 5 waiters, 2 chefs
- System is resilient

This is **loose coupling** - POWERFUL!

## SECTION 3: Create Your Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python test2_py_pkg
```

ROS2 creates:
```
test2_py_pkg/
├── package.xml (metadata)
├── setup.py (entry points)
├── test2_py_pkg/ (source code)
└── ...
```

## SECTION 4: Build and Source

```bash
cd ~/ros2_ws
colcon build --packages-select test2_py_pkg
source install/setup.bash
```

**Critical Rule**: Source workspace in EVERY new terminal!

## SECTION 5: Understanding MyFirstNode

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.counter_ = 0
        
        # Create timer: fires every 1.0 second
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Node started!")
    
    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info(f"Count: {self.counter_}")

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    try:
        rclpy.spin(node)  # Keep running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Timer Concept**: Like an alarm that rings every second

**Spin Concept**: Keep running, processing callbacks

## SECTION 6: Register in setup.py

```python
entry_points={
    'console_scripts': [
        'my_first_node=test2_py_pkg.mynode:main',
    ],
},
```

This makes `ros2 run test2_py_pkg my_first_node` work!

## SECTION 7: Run Your Node

```bash
ros2 run test2_py_pkg my_first_node
```

Output:
```
[INFO] [test_node]: Node started!
[INFO] [test_node]: Count: 1
[INFO] [test_node]: Count: 2
[INFO] [test_node]: Count: 3
```

Stop with Ctrl+C

---

# LESSON 03: ROS2 Topics and Publisher-Subscriber

## OPENING
How do robots communicate? Through Topics! This is how real robots work.

## SECTION 1: What Are Topics?

Topics are **named communication channels**:
- Named: /orders, /sensor_data
- Asynchronous: Publishers don't wait
- Many-to-Many: Multiple publishers and subscribers
- Decoupled: Nodes don't know each other
- Continuous: Perfect for streaming data

## SECTION 2: Restaurant Analogy

**Waiter (Publisher)** announces orders
**/orders (Topic)** communication channel
**Chef (Subscriber)** listens for orders

Why this works:
- Waiter doesn't care if 1 or 10 chefs listen
- Chefs don't care who takes orders
- System is flexible and scalable
- Perfect for real-time data

## SECTION 3: Publisher Node - Waiter

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WaiterNode(Node):
    def __init__(self):
        super().__init__('waiter')
        
        # Create publisher
        self.publisher = self.create_publisher(
            String,      # Message type
            'orders',    # Topic name
            10           # Queue size
        )
        
        self.timer = self.create_timer(1.0, self.publish_order)
        self.order_count = 0
        self.get_logger().info('Waiter started!')
    
    def publish_order(self):
        msg = String()
        self.order_count += 1
        msg.data = f'Order #{self.order_count}: Burger'
        self.publisher.publish(msg)
        self.get_logger().info(f'Announcing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = WaiterNode()
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

**Key Concept - create_publisher**:
```python
self.publisher = self.create_publisher(
    String,   # What message type?
    'orders', # What topic?
    10        # Buffer size
)
```

**Publishing**:
```python
msg = String()
msg.data = 'Order #1: Burger'
self.publisher.publish(msg)  # Send it!
```

## SECTION 4: Subscriber Node - Chef

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChefNode(Node):
    def __init__(self):
        super().__init__('chef')
        
        # Create subscription
        self.subscription = self.create_subscription(
            String,               # Message type
            'orders',             # Topic name
            self.order_callback,  # Callback
            10                    # Queue size
        )
        
        self.get_logger().info('Chef listening...')
    
    def order_callback(self, msg):
        # Automatically called when message arrives!
        self.get_logger().info(f'Received: {msg.data}')
        self.get_logger().info('Cooking...')

def main(args=None):
    rclpy.init(args=args)
    node = ChefNode()
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

**Key Concept - create_subscription**:
```python
self.subscription = self.create_subscription(
    String,               # Message type?
    'orders',             # Topic name?
    self.order_callback,  # Function to call?
    10                    # Buffer size?
)
```

**Callback Magic**:
Whenever a message arrives, this function runs automatically:
```python
def order_callback(self, msg):
    print(f'Got: {msg.data}')
```

## SECTION 5: Register Both Nodes

```python
entry_points={
    'console_scripts': [
        'waiter=test2_py_pkg.waiter:main',
        'chef=test2_py_pkg.chef:main',
    ],
},
```

## SECTION 6: Running Both Nodes

**Terminal 1 - Publisher**:
```bash
ros2 run test2_py_pkg waiter
```

Output:
```
[INFO] [waiter]: Waiter started!
[INFO] [waiter]: Announcing: Order #1: Burger
[INFO] [waiter]: Announcing: Order #2: Burger
```

**Terminal 2 - Subscriber**:
```bash
ros2 run test2_py_pkg chef
```

Output:
```
[INFO] [chef]: Chef listening...
[INFO] [chef]: Received: Order #1: Burger
[INFO] [chef]: Cooking...
[INFO] [chef]: Received: Order #2: Burger
[INFO] [chef]: Cooking...
```

Magic happens: Waiter and Chef communicate!

## SECTION 7: Topic Commands

```bash
ros2 topic list
# Shows: /orders, /parameter_events, /rosout

ros2 topic echo /orders
# Real-time message display

ros2 topic info /orders
# Shows publishers, subscribers, type

ros2 topic hz /orders
# Shows publish frequency (1 Hz = 1 per second)
```

## SECTION 8: Why This Matters

**Real Robot with 20+ Nodes**:
- Vision node: Processes camera
- Movement node: Controls motors
- Planning node: Finds path
- Safety node: Emergency stops

Each runs independently. If one fails, others keep going!

## SECTION 9: Advanced - Multiple Subscribers

Run subscriber multiple times:
```bash
# Terminal 2
ros2 run test2_py_pkg chef

# Terminal 3  
ros2 run test2_py_pkg chef  # Again!
```

Both chefs receive same orders! Flexibility!

---

## Teaching Tips for Educators

1. **Use Analogies**: Restaurant analogy sticks with students
2. **Show Real Output**: Students learn from seeing actual logs
3. **Break Code Into Parts**: Don't dump full code, build it line by line
4. **Live Demo**: Show failures, show fixes
5. **Engagement**: Ask 'Why?' frequently
6. **Practice Problems**: "Create a node that..." assignments
7. **Connect to Real World**: Show industrial robots using ROS2

---

End of Teaching Script
