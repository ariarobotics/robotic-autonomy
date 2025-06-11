—-------------------------------------------------- INSTRUCTIONS—-—--------------------------------------------

Use the template for the following three tutorials:
  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
 https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
 https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

Focus on the following:
* What is **colcon**?
* How do you compile and run different projects?
* Be sure to explain what colcon is, gather all the commands used in these tutorials, and assume Chapter 02 as a prerequisite.

—---------------------------------------------- START OF TUTORIAL —--------------------------------------------



# Chapter 03 – Colcon

> **Goal:** Learn how to install packages with Colcon. This will allow for the usage of different software required for robotics.

---

## Objectives
By the end of this chapter you will be able to:
- Build a Workspace using Colcon
- Install packages with Colcon from any online source

---

## Prerequisites
- Completion of Chapter ‘02`
- ROS 2 Humble installed ([README](../../README.md))  
- 

---

## 1. Why this matters
Colcon is ROS2’s build tool. It’s necessary for compiling and formatting ROS packages inside of a workspace. It’s analogous to Gradle for Java developers or Catkin for those familiar with ROS1.
## 2. Step-by-step

**What is a ROS2 workspace?**
A ROS 2 workspace is a directory that contains all packages (dependencies).
The structure is as follows:
.. code-block:: bash

	ws_name
    └── src
        └── repo_name
            ├── CONTRIBUTING.md
            ├── LICENSE
            ├── rclcpp
            ├── rclpy
            └── README.md

    4 directories, 3 files
4 directories, 3 files

**Creating a workspace**
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
`git clone <GitHub link>

**Underlays vs Overlays**
VERY VERY BRIEF explanation
Build command 

colcon build --symlink-install --parallel-workers 4

Source command
	
---

## 3. Try it


---

## 4. Common errors & fixes (if any)

| Symptom                         | Likely cause                   | Quick fix                          |
|---------------------------------|--------------------------------|------------------------------------|
| Build failed partway            | Ran out of memory| Use ``colcon build`` with ``--parallel-workers <num>`` flag| 

---

## Further reading and references
- Official ROS 2 docs: [Title of section](https://docs.ros.org/en/humble/…)
