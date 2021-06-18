## bcr_teleop

Generic (wasd) keyboard based tele-operation package to publish [geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html) commands. Intended application is to publish commands for non-holonomic mobile robots (Ackermann/Skid-Steer/Differential drives). Publishes commands at 20 Hz.


### Run

Standard Operation,

`rosrun bcr_teleop bcr_teleop_node.py`

Remap to a different topic,

`rosrun bcr_teleop bcr_teleop_node.py cmd_vel:=cmd_vel2`


### Usage

	w: increment linear velocity by 0.1,
    s: decrement linear velocity by 0.1,
    a: increment angular velocity by 0.1,
    d: decrement angular velocity by 0.1,
    space: zero velocity command,
    q: QUIT


#### Published Topics

- cmd_vel: [geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)