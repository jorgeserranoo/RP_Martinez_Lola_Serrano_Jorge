How to run the code:

1. Open a terminal and navigate to the catkin_ws directory.
2. Run the command "catkin_make" to build the project.
3. Run the command "source devel/setup.bash" to source the ROS environment.
4. Run the command roscore so that the nodes can communicate
5. Run the command "rosrun ros_game result_node.py" to launch the result node.
6. Run the command "rosrun ros_game game_node.py" to launch the game node.
7. Run the command "rosrun ros_game control_node_pygame.py" to launch the control node pygame. You can also run "rosrun ros_game control_node.py" to launch the control node and control the starship with terminal inputs and without pygame.
    7.1. If you need to install pygame, run in the terminal the following command: "sudo apt-get install python3-pygame"
8. Run the command "rosrun ros_game info_user.py" to launch the info user node.
9. In the terminal that launches the info user node, you will see a prompt to enter your name, username and age. Do it and press enter.
10. Now you should see the game starting.

If you want to visualize the communication between nodes, use the command "rqt_graph"

In case of wanting to use the command roslaunch to execute everything, due to the way this is programmed, you will still need more than one terminal. In the first one, you will have to run the commmand "roslaunch ros_game game.launch" and in another terminal "rosrun ros_game info_user.py"



