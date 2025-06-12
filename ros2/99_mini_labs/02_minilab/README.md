# Mini-Lab 02 - Chase-the-Leader

**Scenario:**  
Spawn a second turtle that relentlessly chases the first one until it gets tagged.

## Learning goals
* Subscribing to **multiple topics** and combining their data
* Basic 2-D geometry (`atan2`, distance)
* Simple closed-loop control (proportional gains)

## What to build
1. Use the `/spawn` service to create `turtle2` at runtime (hard-code or document the CLI call).  
2. A node (`chaser`) that subscribes to `/turtle1/pose` and `/turtle2/pose`, then publishes `Twist` commands to `/turtle2/cmd_vel` so that:  
   * The angular velocity drives turtle 2 to face turtle 1 (`kp_ang · angle_error`)  
   * The linear velocity drives it forward until the distance < 0.5 units (`kp_lin · distance`)  
3. When turtle 2 "catches" turtle 1, the node should log "**Tag!**" once and stop moving.  
4. Expose gains `kp_ang`, `kp_lin` and the catch distance as parameters.

## Deliverables
* `chaser.py` in a package called `turtle_chaser`
* A `README.md` describing how to spawn the second turtle and run the node
* Screenshot showing both turtles and a console snippet with the “Tag!” message

## References & Hints
* `/spawn` service message definition (`turtlesim/srv/Spawn`)  
* Pose message fields: x, y, theta  
* Remember to wrap angle errors to (-π, π] before multiplying by `kp_ang`
