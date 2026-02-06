# ROS2 Topics 

### Creating a Publisher Node
Navigate to: ~/ros2_ws/src/test2_py_pkg/test2_py_pkg/
Create a new file: `topic_creator.py`



## Registering Your Node

Update `setup.py` with entry points for both nodes:
Edit: ~/ros2_ws/src/test2_py_pkg/setup.py

```python 
entry_points={
    'console_scripts': [
        'my_first_node = test2_py_pkg.mynode:main',
        # ADD THIS LINE:
        'create_topic = test2_py_pkg.topic_creator:main',
    ],
}

```

## Building and Running

### Step 1: Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select test2_py_pkg
```

### Step 2: Source the Workspace

```bash
source install/setup.bash
```

⚠️ **Important**: Source in EVERY new terminal before running nodes!

### Step 3: Run the Nodes (In Separate Terminals)

**Terminal 1 - Run the Waiter (Publisher)**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run test2_py_pkg waiter
```

**Terminal 2 - Run the Chef (Subscriber)**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run test2_py_pkg chef
```

### Expected Output

**Waiter Terminal**:
```
[INFO] [Waiter node started!]
[INFO] [Announcing: Order #1: Burger]
[INFO] [Announcing: Order #2: Burger]
[INFO] [Announcing: Order #3: Burger]
...
```

**Chef Terminal**:
```
[INFO] [Chef node started! Listening for orders...]
[INFO] [Received order: Order #1: Burger]
[INFO] [Starting to cook...]
[INFO] [Received order: Order #2: Burger]
[INFO] [Starting to cook...]
...
```

## Essential ROS2 Topic Commands

### List All Topics

```bash
ros2 topic list
```

**Output**:
```
/orders
/parameter_events
/rosout
```

### Echo Topic Messages in Real-Time

```bash
ros2 topic echo /orders
```

**Output**:
```
data: 'Order #1: Burger'
---
data: 'Order #2: Burger'
---
data: 'Order #3: Burger'
---
```

### Get Topic Information

```bash
ros2 topic info /orders
```

**Output**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### Get Topic Rate and Bandwidth

```bash
ros2 topic hz /orders  # Shows publish frequency
ros2 topic bw /orders  # Shows bandwidth usage
```

### View Topic Data Type

```bash
ros2 interface show std_msgs/msg/String
```

**Output**:
```
string data
```

## Understanding Message Types

### Common Message Types

- **std_msgs/msg/String**: Text messages
- **std_msgs/msg/Int32**: Integer values
- **std_msgs/msg/Float32**: Float values
- **geometry_msgs/msg/Twist**: Robot velocity (linear + angular)
- **sensor_msgs/msg/Image**: Camera images
- **sensor_msgs/msg/LaserScan**: LIDAR data

### Using Different Message Types

**Example with Int32**:
```python
from std_msgs.msg import Int32

self.publisher = self.create_publisher(Int32, 'counter', 10)
msg = Int32()
msg.data = 42
self.publisher.publish(msg)
```

## Advanced: Multiple Publishers and Subscribers

### Multiple Subscribers on One Topic

You can have as many subscribers as you want listening to the same topic:

```bash
# Terminal 1: Publisher
ros2 run test2_py_pkg waiter

# Terminal 2: First Subscriber
ros2 run test2_py_pkg chef

# Terminal 3: Second Subscriber
ros2 run test2_py_pkg chef  # Run chef again

# Terminal 4: Monitor with echo
ros2 topic echo /orders
```

All subscribers receive the same messages independently!

### Multiple Publishers on One Topic

You can also have multiple publishers sending to the same topic:

```python
# Both nodes publish to 'orders'
publisher1 = node1.create_publisher(String, 'orders', 10)
publisher2 = node2.create_publisher(String, 'orders', 10)
```

## Common Issues and Debugging

### Issue: "Topic not created"

Topics are created automatically when the first publisher or subscriber is created. If you don't see a topic:
1. Ensure both publisher and subscriber are running
2. Check node names with `ros2 node list`
3. Verify topic name spelling

### Issue: "Subscriber not receiving messages"

Check:
1. Publisher is running and publishing
2. Topic names match exactly (case-sensitive)
3. Message types match
4. Use `ros2 topic echo /topic_name` to verify messages are being published

### Issue: "No messages from subscriber"

Add logging:
```python
def callback(self, msg):
    self.get_logger().info(f'Message received: {msg.data}')
    # Without logging, you won't see if callback is called
```

### Debugging Commands

```bash
# List all active nodes
ros2 node list

# Show information about a node
ros2 node info /waiter

# List all topics
ros2 topic list

# Show topic details
ros2 topic info /orders

# View topic messages in real-time
ros2 topic echo /orders

# Show publish rate
ros2 topic hz /orders
```

## Practice Exercise

### Create a Temperature Sensor Publisher

1. Create a new file: `src/test2_py_pkg/test2_py_pkg/temperature_sensor.py`
2. Implement a node that:
   - Publishes temperature data to a topic `/temperature`
   - Uses Float32 message type
   - Simulates temperature readings (e.g., 20.0°C starting, increasing by 0.1 each second)
   - Logs each published value
3. Create a temperature monitor subscriber that listens and logs temperature
4. Run both and verify they communicate

## Comparison: Topics vs Services vs Actions

| Feature | Topics | Services | Actions |
|---------|--------|----------|----------|
| **Pattern** | Pub-Sub | Request-Response | Asynchronous Goal |
| **Communication** | Asynchronous | Synchronous | Asynchronous with feedback |
| **Type** | Many-to-Many | One-to-One | One-to-One |
| **Best for** | Continuous data (sensors) | Occasional requests | Long-running tasks |
| **Example** | Temperature streaming | Request calculation | Robot navigation |

## Summary

- **Topics** enable asynchronous, many-to-many communication
- **Publishers** send data to topics without knowing about subscribers
- **Subscribers** listen to topics and react to arriving messages
- Topics are created automatically by ROS2
- The Pub-Sub pattern decouples nodes, making systems flexible and scalable
- Use `ros2 topic` commands to inspect and debug topics
- Message types define the structure of data (String, Int32, custom types, etc.)

## Next Lesson

In the next lesson, we'll explore **ROS2 Services** and implement the **Request-Response pattern** for synchronous communication between nodes.
