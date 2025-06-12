# Chapter 03 – Colcon

> **Goal:** Learn how to install packages with Colcon. This will allow for the usage of different software required for robotics.

## Objectives
By the end of this chapter you will be able to:
- Build a Workspace using Colcon
- Install packages with Colcon from any online source

## Prerequisites
- Completion of Chapter ‘02`
- ROS 2 Humble installed ([README](../../README.md))  

## 1. Why this matters
Colcon is ROS2’s build tool. It’s necessary for compiling and formatting ROS packages inside of a workspace. It’s analogous to Gradle for Java developers or Catkin for those familiar with ROS1.
## 2. Step-by-step

**What is a ROS2 workspace?**
A ROS 2 workspace is a directory that contains all packages (dependencies).
The structure is as follows:

```
ws_name
    └── src
        └── repo_name
            ├── CONTRIBUTING.md
            ├── LICENSE
            ├── rclcpp
            ├── rclpy
            └── README.md
    4 directories, 3 files

```


**Creating a workspace**
Use the following commands to cd into the workspace directory and clone the files needed for building the package.
```
mkdir -p ~/ros2_ws/src 
cd ~/ros2_ws
git clone <github_link>
```

**Underlays vs Overlays**
Sourcing is the act of making the ROS2 interface and packages available to the CLI.
Underlays are the existing packages that workspace runs on (eg. ROS 2)
Overlays are the packages that the workspace brings in itself.
For simplicity, we’ll be sourcing the basic ros2 install as our underlay for this tutorial:
``
source /opt/ros/humble/setup.bash``

**Build command** 
After creating the structure described above, you may now finally build the package using the following command. Adjust the value following the ``
--parallel workers``
 flag to adjust allocated memory as needed.
``
colcon build --symlink-install --parallel-workers 2``


Source command
Got to the root workspace directory first.

``source install/setup.bash``


## 3. Try it


---

## 4. Common errors & fixes (if any)

| Symptom                         | Likely cause                   | Quick fix                          |
|---------------------------------|--------------------------------|------------------------------------|
| You have a Fatal Error Build failed partway          | Ran out of memory|    Use ``colcon build`` with ``--parallel-workers <num>``flag| 

---

## Further reading and references
- Official ROS 2 docs: [Title of section](https://docs.ros.org/en/humble/…)


