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

Client (Waiter)
    |
    v
[Request] → Server (Kitchen Manager)
    ^
    |
[Response]

Client WAITS here until server responds!


## Server Creating a Service Interface
## Step 1: Create a new package for service definitions

cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_interfaces
# ament_cmake for C++ interface packages (even if we use from Python)


## Step 2: Create the srv directory


## Server Implementation (Python)


## Client Implementation (Python)


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
