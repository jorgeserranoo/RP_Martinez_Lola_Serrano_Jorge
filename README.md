# ROS Galaga Arcade Game

Final project for the **Robot Programming** course at Universidad Carlos III de Madrid. A Galaga-inspired arcade game built with **Python + pygame**, integrated with **ROS 1 (Noetic)** using topics, services and parameters.

UC3M · Robotics Engineering · 2024

---

## Overview

The game runs as a ROS node graph with three main nodes:

| Node | Role |
|---|---|
| `INFO_USER` | Requests player info and publishes it to the game node |
| `GAME_NODE` | Main game logic — welcome screen, gameplay, game over |
| `RESULT_NODE` | Receives player info and final score, displays results |

Services expose score retrieval (`GET_GAME_SCORE`) and difficulty control (`SET_GAME_DIFFICULTY`).

---

## Branches

| Branch | Description |
|---|---|
| `ROS` | Full ROS Noetic integration — requires a ROS environment |
| `Game` | Standalone pygame version — no ROS needed |

---

## Requirements

**Game branch (standalone):**
```bash
pip install pygame numpy
python game.py
```

**ROS branch:**
```bash
# ROS Noetic required
roslaunch <package> game.launch
```

---

## Authors

Jorge Serrano Navas · Lola Martínez  
UC3M · 2024
