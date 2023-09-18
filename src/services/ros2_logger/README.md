# ROS2 Logger Service

Takes [ROS log messages](https://docs.ros2.org/foxy/api/rcl_interfaces/msg/Log.html) from a configurable ros topic and maps them based upon ROS severity level to Viam log types. Additionally you can dynamically reconfigure the detail level of log entries sent to the Viam cloud.


## Service Configuration
Once the ROS2 Module is deployed, add the service part within services below to your Viam services configuration:

```
  "services": [
    {
      "name": "mylogger",
      "type": "ros2_logger",
      "namespace": "viamlabs",
      "model": "viamlabs:ros2:ros2_logger",
      "attributes": {
        "ros_topic": "/rosout",
        "log_level": "error"
      }
    }
  ]
```

## Publish Log Entry from Command Line
Change the level according to your needs:
debug = 10
info  = 20
warn  = 30
error = 40
critical = 50

```ros2 topic pub -1 /rosout rcl_interfaces/msg/Log "{msg: 'Hello from terminal', level: 40}"```
