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

### Step 3: Run the Node


```bash
ros2 run test2_py_pkg create_topic
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
