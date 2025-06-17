*Running Our Code*
First run ``source /opt/ros/humble/setup.bash`` to properly configure your ros2 underlay. Then, from the workspace root, run ``colcon build --packages-select teleop_remap`` to build this project, and run ``. install/setup.bash`` to add the entrypoint for your code. Then, you will need to run the following commands in the following order, all from different terminals. Each command creates a different ros2 node
1. ``ros2 run turtlesim turtlesim_node``
2. ``ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/turtle1/teleop_raw``
3. ``ros2 run teleop_remap talker_listener``
Finally, in a new terminal, you can edit some ros parameters to change how to remapper node interprets your keyboard input. Run ``ros2 param set remapper_node invert_angular <some_bool_val>`` to invert/revert the direction controls and/or ``ros2 param set remapper_node linear_scale <some_float_val>`` to adjust/invert the turtle's speed.
Yous can also run ``ros2 topic echo /turtle1/<some_topic>`` to compare the raw data from turtle_teleop_key to the remapped data.