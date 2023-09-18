# viam-ros2-module

This is a simple viam-ros integration which has been tested with ROS2 Humble turtlebot4 & turtlebot4-lite

## Overview


## How To Use
1. Install Viam on ROS2 Humble robot
2. clone repo on robot using `git clone`
3. setup virtual environment using `setup.sh`
4. configure ROS2 module in https://app.viam.com
5. configure each component (see [sample_config.json](./sample_configs/sample_config.json) as reference)

## Viam running as root and rmw_fastrtps_cpp
By default, rmw_fastrtps_cpp uses [eProsima](https://www.eprosima.com/index.php) DDS implementation, 
which makes use of [shared memory transports](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html)
when communication is between entities running on the same processing unit (or host).

Since viam by default starts as root, there are some shared memory issues. The current fix is to update
the DDS implementation to use `UDPv4` as the only transport, see [fastdds_rpi.xml](./sample_configs/fastdds_rpi.xml) for 
the example config used in turtlebot4.

## TODO

### Overall
1. update sensor to use single viam node
2. create test cases for each message type supported
3. improve documentation
4. add message
5. fix ros_base (only publish when needed - currently sending all the time)
6. set parameter config
7. ability to build custom messaging

### turtlebot specific
1. service calls such as dock etc.
2. more fun stuff

# References
1. [viam](https://docs.viam.com)
2. [humble](https://docs.ros.org/en/humble/index.html)
3. [turtlebot4](https://clearpathrobotics.com/turtlebot-4/)
4. [turtlebot4 user guide](https://turtlebot.github.io/turtlebot4-user-manual/)

# Contact

Any issues feel free to open an issue on the github, create pull requests, etc.