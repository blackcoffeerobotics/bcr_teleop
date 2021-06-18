## bcr_teleop

Generic keyboard based tele-operation package to publish [geometry_msgs/Twist](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Twist.msg) commands. Intended application is to publish commands for non-holonomic mobile robots (Ackermann/Skid-Steer/Differential drives). Publishes commands at 20 Hz.


### Run

Standard Operation,

`ros2 run bcr_teleop bcr_teleop_node`

Remap to a different topic,

`ros2 run bcr_teleop bcr_teleop_node --ros-args --remap cmd_vel:=cmd_vel2`


### Usage

	w: increment linear velocity by 0.1,
    s: decrement linear velocity by 0.1,
    a: increment angular velocity by 0.1,
    d: decrement angular velocity by 0.1,
    space: zero velocity command,
    q: QUIT


#### Published Topics

- cmd_vel: [geometry_msgs/Twist](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Twist.msg)