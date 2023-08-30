# ROS2 Logger Service

Takes [ROS log messages](http://docs.ros.org/en/api/rosgraph_msgs/html/msg/Log.html) from a configurable ros topic and maps them based upon ROS severity level to Viam log types. Additionally you can dynamically reconfigure the detail level of log entries sent to the Viam cloud.


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
