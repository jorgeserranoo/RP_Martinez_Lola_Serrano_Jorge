How to run the code:

1. Open a terminal and navigate to the catkin_ws directory.
2. Run the command "catkin_make" to build the project.
3. Run the command "source devel/setup.bash" to source the ROS environment.
4. Run the command "rosrun ros_game result_node.py" to launch the result node.
5. Run the command "rosrun ros_game game_node.py" to launch the game node.
6. Run the command "rosrun ros_game control_node_pygame.py" to launch the control node pygame. You can also run "rosrun ros_game control_node.py" to launch the control node with the keyboard and without pygame.
    6.1. If you need to install pygame, run in the terminal the following command: "sudo apt-get install python3-pygame"
7. Run the command "rosrun ros_game info_user.py" to launch the info user node.
8. In the terminal that launches the info user node, you will see a prompt to enter your name, username and age. Do it and press enter.
9. Now you should see the game starting.



