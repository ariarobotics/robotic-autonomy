This is an introduction to basic ROS2 commands.
The keyword ‘ros2‘ is the unique entry point for the CLI. Every ROS 2 command starts with the ros2 keyword, followed by a command, a verb, and possibly positional/optional arguments.
 ros2 <command> <verb>

General Commands
-
ros2 <node/topic/service> type
  -Finds out the type of a node, topic, or service

ros2 interface show <type>
  -Displays interface definition

ros2 run <package_name> <executable_name> __node:=<new_node_name> <old_name>:=<new_name>
  -It allows you to change the name of a topic, node or service therefore modifying the node.
eg. In turtlesim the command ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel change the name of the topic to modify the node that is making the other turtle (turtle2) that was spawned using /spawn work separately than the other turtle (turtle1, the original)

ros2 <node/topic/service> list
  -Shows the list of all the nodes, topics, or services in the system
  -Adding “-t” will also show the types of each of the nodes/topics/services in the list

ros2 bag record <topic_name>
  -Records the data published to a topic
  -Adding “-o” allows the user to name the recording

ros2 <node/topic/service> info
  -returns a list of subscribers, publishers, services, and actions

ros2 <node/topic/service> find <type>
  -Returns a list of nodes/topics/services with a specific type


Nodes:
-
ros2 run <package> <executable>
  -ros2 run launches an executable from a package in this case, eg. turtlesim is the package

Topics:
-
ros2 topic pub <topic> <type> <values>
  -”--once” will publish to the topic only once
  -”--rate <frequency>” will publish to the topic at a specific frequency

ros2 topic echo <topic>  eg. ros2 topic echo /turtle1/cmd_vel
  -This will print data sent via a topic to the command line

ros2 topic hz <topic>
  -Allows you to see the rate at which the data is being published

ros2 topic bw <topic>
  -This prints the bandwidth of a topic

Services:
-
ros2 service call <service> <type> <values>
  -Allows you to call a service

Params:
-
ros2 param get <node> <parameter>
  -Displays the current type and value of a parameter

ros2 param set <node> <parameter> <value>
  -This sets values for the parameters of a node

ros2 param dump <node>
  -Allows you to view all of a node’s parameter values

ros2 param load <node> <file>
  -Allows you to load parameters from a file to a running node
 
Actions:
-
ros2 action send_goal <action> <type> <values>
  -Sends a goal to an action server; the user (or server) may choose to cancel the action
