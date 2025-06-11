# Chapter 00 – Introduction

> **Goal:** By the end of this chapter you should be familiar with the ROS2 fundemental concepts and their core commands.

---

## Objectives
By the end of this chapter you will be able to:
- Understand Nodes and its basics
- Understand Topics and its basics
- Understand Services and its basics
- Understand Parameters
- Understand Actions and its basics

---

## Prerequisites
- ROS 2 Humble installed ([README](../../README.md))  

---

## 1. Why this matters
All of the above topics mentioned in objectives make up what is ROS2. These topics are fundemental to using ROS2 in a powerful way and having a greater understanding of these will prove to be a useful tool in robotic applications. They are the building blocks of robotic software.

---

## 2. Nodes

1. **Command to run specfic program from a ROS 2 pacage**  
   ```bash
   # ros2 run <package_name> <executable_name>
   ```
   The ___ros2 run <package_name> <executable_name>___ command launches a pre-built node (executable) from a specified __ROS 2__ package. It's used to start a specifc part of your robot's software, like a sensor or controller, that's aldready been complied. __Pitfall__: If the pacage isin't built or sourced properly (i.e., you didn't run source _install/setup.bash_), it wont find the package or executable.

2. **Command to launch the Turtlesim program and its main node** 
   ```bash
   # ros2 run turtlesim turtlesim_node
   ```
   The command ___ros2 run turtlesim turtlesim_node___ starts the turtlesim program by running the turtlesim_node from the turtlesim package.

3. **Command to check multiple node lists**  
   ```bash
   # ros2 node list /turtlesim
   ```
   The ros2 node list command shows all active ROS 2 nodes currently running, which helps you keep track of your robot's part. __Pitfalls__: If no nodes are listed, it likley means no nodes are running (or the terminal hasn't sourced he __ROS 2__ setup file.) Also running this is in a terminal that doesn't properly source __ROS 2__, won't display anything.


   ```bash
   # ros2 run turtlesim turtle_teleop_key
   ```

   When you run __ros2 run turtlesum turtlesim_node__, it starts the ___/turtlesim_ node__, and running ros2 run __turtlesim turtle_teleop_key__ starts another node called __/teleop_turtle__. By running ___ros2 node list___ again, you'll see both nodes listed, confirming that they're up and running. __Notes__: Use the arrow keys to move turtle in real time! Great for testing node inetraction and topic publishing (it sends velocity commands to __/turtle1/cmd_vel__).
   
   ```bash
   # ros2 node list /turtlesim /teleop_turtle
   ```
   This command runs the __turtle_teleop_key__ executable from the turtlesim package, which lets you control the turtle with your keyboard. When you check ros2 node list again, you’ll see both the __/turtlesim__ and __/teleop_turtle__ nodes running.
   

4. **Remapping**  
   ```bash
   # ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
   ```
   The command __ros2 run turtlesim turtlesim_node --ros-args --remap__ __node:=my_turtle__ runs the turtlesim node but changes its default name from __/turtlesim__ to __/my_turtle__. This lets you run multiple turtlesim nodes without name conflicts or customize the node name for easier identification. You can verify the change by running ros2 node list, which will now show __/my_turtle__ instead of __/turtlesim__.

5. **Examination of nodes**  
   ```bash
   # ros2 node info /my_turtle
   ```
   This command displays all the communication interfaces associated with the __/my_turtle node__. It shows which topics the node subscribes to (e.g., __/turtle1/cmd_vel__), publishes to (like __/turtle1/pose__), and the services and actions it provides or uses (like __/spawn__, __/reset__, __/turtle1/rotate__ absolute). This helps visualize how __/my_turtle__ interacts with the rest of the ROS system.

---
## 3. Topics

1. **Nodes**  
   ```bash
   # code or command here
   ```
   _Explanation: what’s happening and why it matters._

2. **Topics**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

3. **Services**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

4. **Parameters**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

5. **Actions**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

---


## 3. Services

1. **Nodes**  
   ```bash
   # code or command here
   ```
   _Explanation: what’s happening and why it matters._

2. **Topics**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

3. **Services**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

4. **Parameters**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

5. **Actions**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

---

## 4. Try it

> A brief exercise to reinforce the chapter—what to run or modify next.

```bash
# e.g. invoke a node, tweak a param, view output, etc.
```

---

## 5. Common errors & fixes (if any)

| Symptom                         | Likely cause                   | Quick fix                          |
|---------------------------------|--------------------------------|------------------------------------|
| `command not found`             | You forgot to `source` the setup script. | `source install/setup.bash`        |
| Node crashes on startup         | Bad parameter file or YAML indentation.   | Check syntax/indentation in your params file. |


---

## Further reading and references
- Official ROS 2 docs: [Title of section](https://docs.ros.org/en/humble/…)
- Any other link you have used and though is good.
