# Lesson 05: ROS2 Services - Request/Response Communication

## Quick Summary

Services provide **synchronous, blocking** communication for occasional requests, unlike Topics which are asynchronous and continuous.

## Key Difference: Topics vs Services

| Aspect | Topics | Services |
|--------|--------|----------|
| **Pattern** | Publish-Subscribe (async) | Request-Response (sync) |
| **Participants** | Many-to-Many | Client-Server |
| **Blocking** | No | Yes - client waits |
| **Use Case** | Sensor streams | Commands, calculations |

## Service Definition (.srv file)

```
int64 a
int64 b
---
int64 sum
```

**Format**: Request fields above `---`, response fields below


## ROS2 Service Flow
```
Client (Waiter)
    |
    v
[Request] → Server (Kitchen Manager)
    ^
    |
[Response]

Client WAITS here until server responds!
```

# Server Creating a Service Interface
## Step 1: Create a new package for service definitions
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_interfaces
```
## Step 2: Create the srv directory
```bash
mkdir my_interfaces/srv
```
## Step 3: Create the service definition file
```bash
Navigate to File: ~/ros2_ws/src/my_interfaces/srv/TakeOrder.srv
```

```bash
# REQUEST (what the client sends)
string item_name
int32 quantity
---
# RESPONSE (what the server sends back)
bool accepted
string message
```
## Configure package files
```
Edit: ~/ros2_ws/src/my_interfaces/package.xml
Edit: ~/ros2_ws/src/my_interfaces/CMakeLists.txt
```
## Build the service interface
```bash
cd ~/ros2_ws
colcon build --packages-select my_interfaces
source install/setup.bash
```

## Verify it was created:
```bash
ros2 interface list | grep TakeOrder
# Output: my_interfaces/srv/TakeOrder
```

## Creating the Service Server
```bash
File: ~/ros2_ws/src/test2_py_pkg/test2_py_pkg/restaurant_server.py
```
## Creating the Service Client
```
File: ~/ros2_ws/src/test2_py_pkg/test2_py_pkg/restaurant_client.py
```
## Register Service Nodes
```bash
# Edit: ~/ros2_ws/src/test2_py_pkg/setup.py

entry_points={
    'console_scripts': [
        'my_first_node = test2_py_pkg.mynode:main',
        'create_topic = test2_py_pkg.topic_creator:main',
        'waiter = test2_py_pkg.waiter:main',
        'chef = test2_py_pkg.chef:main',
        # ADD THESE LINES:
        'restaurant_server = test2_py_pkg.restaurant_server:main',
        'restaurant_client = test2_py_pkg.restaurant_client:main',
    ],
}

```

# Testing the Service
## Step 1: Build both packages

```
cd ~/ros2_ws
colcon build --packages-select my_interfaces test2_py_pkg
source install/setup.bash
```
## Step 2: Terminal 1 - Start Server
```
ros2 run test2_py_pkg restaurant_server

# Expected output:
text
[INFO] 🍽️  Restaurant is OPEN! Waiting for orders...

```
## Step 3: Terminal 2 - Send Requests
```
# First request: 2 burgers
ros2 run test2_py_pkg restaurant_client burger 2

# Output should show:
# [INFO] 🍽️  Placing order: 2x burger
# [INFO] ✅ Delicious! 2 burgers coming up!

# In Terminal 1, the server shows:
# [INFO] 📋 Incoming request: 2x burger
# [INFO] 📤 Sending response: ✅ Delicious! 2 burgers coming up!
```


## CLI Commands

```bash
# List services
ros2 service list

# Call service from command line
ros2 service call /add example_interfaces/srv/AddTwoInts '{a: 10, b: 5}'

# Get service info
ros2 service info /add

# Find service type
ros2 service type /add
```

## Next: Lesson 06 - Launch Files

Use launch files to start multiple nodes with one command!
