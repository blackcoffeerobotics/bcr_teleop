## bcr_teleop

Keyboard based teleoperation package to publish geometry_msgs/Twist commands. Intended application is to publish commands for non-holonomic mobile robots (Ackermann/Skid-Steer/Differential drives). 

`rosrun bcr_teleop bcr_teleop_node.py`

This is a w/a/s/d game type node, increases/decreases, linear/angular velocities by press of w/s,a/d keys.
