# Mini-Lab 00 - Spirograph Turtle

**Scenario:**  
Turn *turtlesim* into a poor-man's spirograph: publish velocity commands that make the turtle draw pretty parametric curves.

## Learning goals
* Publishing on a fixed **timer**
* Converting simple math (polar coordinates) into linear & angular velocity
* Calling basic **services** to change pen colour or clear the screen

## What to build
1. A Python node (`spirograph`) that publishes `Twist` messages to `/turtle1/cmd_vel` at 50 Hz.  
2. Implement at least two curve types:  
   * Circle *(sanity check)*  
   * Rose curve *r(θ) = a·cos(kθ)* (choose **k** as a parameter)  
3. Expose the following **parameters** (all live-tunable):  
   `radius` (float), `petals` (int, only for rose), `speed` (float θ̇), `pen_color` (tuple r,g,b).  
4. On startup (or when a parameter changes) the node should call `/turtle1/set_pen` to update the pen colour.

## Deliverables
* `spirograph.py` in a package called `spirograph`
* A `README.md` that documents the parameter interface and shows two screenshots (circle & rose)
* A short GIF or screenshot demostrating the reults

## References
* Rose curve math - <https://mathworld.wolfram.com/RoseCurve.html>
* Publisher tutorial - <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>
