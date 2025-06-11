# Chapter 02 – Workspaces & Packages

## Objectives
By the end of this chapter you will be able to:
- Set up a ROS2 workspace
- Create a package inside of your workspace
---

## Prerequisites
- Completion of Chapter ‘01`
- ROS 2 Humble installed ([README](../../README.md))  
---

## 1. Why this matters
A workspace is where you’ll write code and import repositories for your ROS2 project.
## 2. Step-by-step

**What is a ROS2 workspace?**
A ROS 2 workspace is a directory that contains all packages (dependencies).

	ws_name
    └── src
        └── repo_name
            ├── CONTRIBUTING.md
            ├── LICENSE
            ├── rclcpp
            ├── rclpy
            └── README.md

    4 directories, 3 files



**Creating a workspace**
Create the basic workspace structure, with an src directory inside of your workspace root

``` mkdir -p ~/ros2_ws/src ```
``` cd ~/ros2_ws ```



**Underlays vs Overlays**
Underlays are the existing packages that workspace runs on (eg. ROS 2)
Overlays are the packages that the workspace brings in itself.
For simplicity, we’ll be sourcing the basic ros2 install as our underlay for this tutorial:
``source /opt/ros/humble/setup.bash``



Build command 

colcon build --symlink-install --parallel-workers 4

Source command

source install/setup.bash
	
---

## 3. Try it
cd src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package
cd ..
colcon build
source install/local_setup.bash
ros2 run my_package my_node

Do you see the hello world?

---

## 4. Common errors & fixes (if any)

| Symptom                         | Likely cause                   | Quick fix                          |
|---------------------------------|--------------------------------|------------------------------------|
<<<<<<< HEAD
| Build failed partway            | Ran out of memory|             |Use ``colcon build`` with ``--parallel-workers <num>`` flag| 
=======
| Build failed partway            | Ran out of memory| Use ``colcon build`` with ``--parallel-workers <num>`` flag| 
>>>>>>> f2201d5 (test)

---

## Further reading and references
- Official ROS 2 docs: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#source-the-setup-file
