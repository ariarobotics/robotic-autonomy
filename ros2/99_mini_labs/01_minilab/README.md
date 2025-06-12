# Mini-Lab 01 - Teleop Remap ("Opposite Day")

**Scenario:**  
You already know how to drive *turtlesim* with `turtle_teleop_key`.  
Your job is to slip a **relay node** in the middle that *messes with* the commands before the turtle sees them.

## Learning goals
* Topic **remapping** and introspection with the ROS 2 CLI  
* Writing a minimal **subscriber -> publisher** Python node (`rclpy`)  
* Using **runtime parameters** so behaviour can be tuned on-the-fly

## What to build
1. A Python node (`teleop_remap`) that  
   * subscribes to a `geometry_msgs/msg/Twist` topic you pick (e.g. `/teleop_raw`),  
   * applies a transform to the message (at minimum: scale linear speed and optionally invert angular velocity),  
   * republishes the result to `/turtle1/cmd_vel`.
2. At launch time, you must **remap** the output of `turtle_teleop_key` to the input your node expects - no editing of the teleop code allowed.
3. Expose at least two parameters: `linear_scale` (float) and `invert_angular` (bool).  They should be live-tunable with `ros2 param set`.

## Deliverables
* `teleop_remap.py` in a package called `teleop_remap`
* A one-page `README.md` explaining **how to run** your setup and **which parameters** you implemented
* A short GIF or screenshot that proves the turtle behaves differently than vanilla teleop at the end.


## Resrouces 
* [`geometry_msgs/msg/Twist`](https://docs.ros.org/en/humble/p/geometry_msgs/msg/Twist.html)
