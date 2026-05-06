================================================================
README — RP_Martinez_Lola_Serrano_Jorge (ROS Galaga)
ROS Galaga Arcade Game
Final project for the Robot Programming course at Universidad Carlos III de Madrid. A Galaga-inspired arcade game built with Python + pygame, integrated with ROS 1 (Noetic) using topics, services and parameters.
UC3M · Robotics Engineering · 2024

Overview
The game runs as a ROS node graph with three main nodes:
NodeRoleINFO_USERRequests player info and publishes it to the game nodeGAME_NODEMain game logic — welcome screen, gameplay, game overRESULT_NODEReceives player info and final score, displays results
Services expose score retrieval (GET_GAME_SCORE) and difficulty control (SET_GAME_DIFFICULTY).

Branches
BranchDescriptionROSFull ROS Noetic integration — requires a ROS environmentGameStandalone pygame version — no ROS needed

Requirements
Game branch (standalone):
bashpip install pygame numpy
python game.py
ROS branch:
bash# ROS Noetic required
roslaunch <package> game.launch

Authors
Jorge Serrano Navas · Lola Martínez
UC3M · 2024
