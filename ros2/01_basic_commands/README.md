# Chapter 01 - Basic Commands

> **Goal:** Use introductory knowledge of nodes, topics, services, and actions to be able to effectively learn and use basic ros2 commands.

---

## Objectives
By the end of this chapter you will be able to:
- …run ros2 packages
- …manipulate attributes of the /turtlesim node by publishing data to topics, calling services, and sending goals to action servers.
- …print and format data to the command line using the basic commands of ros2 topics
- …create, save, and open recordings of data of a defined time interval sent through a topic
- …get, set, save, and load ros2 parameters

---

## Prerequisites
- Completion of Chapter `<00−Introduction>` 
- ROS 2 Humble installed ([README](../../README.md))  
- X11 server set up from host (if using dev container)  

---

## 1. Why this matters
_Being able to use basic ros2 commands is necessary to be able to use packages and manipulate or read from the ROS graph.

---

## 2. Step-by-step

1. **Command Structure**

   ```bash
   # ros2 <command> <verb>
   ```
The keyword ‘ros2‘ is the unique entry point for the CLI. Every ROS 2 command starts with the ros2 keyword, followed by a command, a verb, and possibly positional/optional arguments._

3. **General Commands**
   
   ```bash
      #ros2 <node/topic/service> type
   ```
Finds out the type of a node, topic, or service

   
   ```bash
   #ros2 interface show <type>
   ```
Displays interface definition


   ```bash
   #ros2 run <package_name> <executable_name> __node:=<new_node_name> <old_name>:=<new_name>
   ```
It allows you to change the name of a topic, node or service therefore modifying the node.
eg. In turtlesim the command ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel change the name of the topic to modify the node that is making the other turtle (turtle2) that was spawned using /spawn work separately than the other turtle (turtle1, the original)


   ```bash
   #ros2 <node/topic/service> list
   ```
  Shows the list of all the nodes, topics, or services in the system
  Adding “-t” will also show the types of each of the nodes/topics/services in the list


   ```bash
   #ros2 bag record <topic_name>
   ```
  Records the data published to a topic
  Adding “-o” allows the user to name the recording


   ```bash
   #ros2 <node/topic/service> info
   ```
  Returns a list of subscribers, publishers, services, and actions


   ```bash
   #ros2 <node/topic/service> find <type>
   ```
  Returns a list of nodes/topics/services with a specific type

3. **Node Commands**  
   ```bash
   #ros2 run <package> <executable> -ros2 run launches an executable from a package in this case, eg. turtlesim is the package
   ```
   _These commands are specific to nodes.._

4. **Topic Commands**  
   ```bash
   # ros2 topic pub <topic> <type> <values>
  -”--once” will publish to the topic only once
  -”--rate <frequency>” will publish to the topic at a specific frequency

ros2 topic echo <topic>  eg. ros2 topic echo /turtle1/cmd_vel
  -This will print data sent via a topic to the command line

ros2 topic hz <topic>
  -Allows you to see the rate at which the data is being published

ros2 topic bw <topic>
  -This prints the bandwidth of a topic

5. **Topic Commands** 
   ```bash
ros2 service call <service> <type> <values>
  -Allows you to call a service
   ```
   _These commands are specific to topics.._

6. **<Param Commands>**  
   ```bash
   # ros2 param get <node> <parameter>
  -Displays the current type and value of a parameter

ros2 param set <node> <parameter> <value>
  -This sets values for the parameters of a node

ros2 param dump <node>
  -Allows you to view all of a node’s parameter values

ros2 param load <node> <file>
  -Allows you to load parameters from a file to a running node
   ```
   _These commands are specific to params.._

7. **<Action Commands>**  
   ```bash
   # ros2 run <package> <executable> -ros2 run launches an executable from a package in this case, eg. turtlesim is the package
   ```
   _These commands are specific to actions.._

---



## 3. Try it

> A brief exercise to reinforce the chapter—what to run or modify next.

```bash
# ros2 action send_goal <action> <type> <values>
  -Sends a goal to an action server; the user (or server) may choose to cancel the action
```

---

## 4. Common errors & fixes

| Symptom                     	| Likely cause               	| Quick fix                      	|
|---------------------------------|--------------------------------|------------------------------------|
| `command not found`         	| You forgot to `source` the setup script. | `source install/setup.bash`    	|
| Node crashes on startup     	| Bad parameter file or YAML indentation.   | Check syntax/indentation in your params file. |


---

## Further reading and references
- Official ROS 2 docs: [Title of section](https://docs.ros.org/en/humble/…)
