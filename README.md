# Traveling Ethiopia Search Problem

Solutions for the Traveling Ethiopia Search Problem covering uninformed search, uniform cost search, A* search, adversarial MiniMax search, and ROS 2-based robot navigation in Gazebo.

---

## Table of Contents

- [Project Overview](#project-overview)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Running Questions 1-4](#running-questions-1-4)
- [Question 5: ROS 2 Robot Navigation](#question-5-ros-2-robot-navigation)
  - [Installation](#installation)
  - [Building the ROS 2 Package](#building-the-ros-2-package)
  - [Running the Simulation](#running-the-simulation)
  - [Launch Arguments](#launch-arguments)
  - [Available Cities (Figure 5)](#available-cities-figure-5)
  - [Robot Specifications](#robot-specifications)
- [Algorithm Details](#algorithm-details)
- [Troubleshooting](#troubleshooting)

---

## Project Overview

| Question | Description | Module | Figure |
|----------|-------------|--------|--------|
| 1.1 | Convert state space graph to data structure | `data/graph_unweighted.py` | Figure 1 |
| 1.2 | BFS and DFS search strategies | `uninformed_search/` | Figure 1 |
| 2.1 | Convert weighted graph to data structure | `data/graph_weighted.py` | Figure 2 |
| 2.2 | UCS: Addis Ababa to Lalibela | `uniform_cost_search/ucs.py` | Figure 2 |
| 2.3 | Multi-goal UCS visiting 8 cities | `uniform_cost_search/multi_goal_ucs.py` | Figure 2 |
| 3 | A* Search: Addis Ababa to Moyale | `astar_search/astar.py` | Figure 3 |
| 4 | MiniMax adversarial search for coffee quality | `minmax_search/minmax.py` | Figure 4 |
| 5.1 | Three-wheel robot with sensors in Gazebo | `ros_navigation/worlds/ethiopia.world` | Figure 5 |
| 5.2 | World file with all 41 states | `ros_navigation/worlds/ethiopia.world` | Figure 5 |
| 5.3 | ROS 2 BFS navigation controller | `ros_navigation/ethiopia_navigation/ethiopia_controller.py` | Figure 5 |

---

## Project Structure

```
traveling_ethiopia/
├── data/                                # Shared graph data for all questions
│   ├── graph_unweighted.py              # Figure 1: unweighted adjacency list
│   ├── graph_weighted.py                # Figure 2: weighted adjacency dict
│   ├── graph_relaxed.py                 # Figure 5: relaxed graph (41 cities)
│   ├── heuristic.py                     # Figure 3: heuristic values for A*
│   └── adversarial_graph.py             # Figure 4: adversarial game tree
├── figures/                             # Reference images (Figures 1-5)
├── uninformed_search/                   # Question 1
│   ├── bfs.py                           # Breadth-First Search
│   ├── dfs.py                           # Depth-First Search
│   ├── search_agent.py                  # SearchAgent class (BFS/DFS)
│   └── demo.py                          # Demo: Addis Ababa to Lalibela
├── uniform_cost_search/                 # Question 2
│   ├── ucs.py                           # Uniform Cost Search
│   ├── multi_goal_ucs.py               # Multi-goal UCS (greedy nearest)
│   ├── demo.py                          # Demo: single goal (Lalibela)
│   └── demo_multi_goal.py              # Demo: 8 goal cities
├── astar_search/                        # Question 3
│   ├── astar.py                         # A* Search (AStarAgent class)
│   └── demo.py                          # Demo: Addis Ababa to Moyale
├── minmax_search/                       # Question 4
│   └── minmax.py                        # MiniMax search for coffee quality
├── ros_navigation/                      # Question 5 (ROS 2 + Gazebo)
│   ├── ethiopia_navigation/             # ROS 2 Python package
│   │   ├── __init__.py
│   │   └── ethiopia_controller.py       # BFS path planning + navigation node
│   ├── launch/
│   │   └── ethiopia_navigation.launch.py
│   ├── worlds/
│   │   └── ethiopia.world               # Gazebo world with 41 cities
│   ├── urdf/
│   │   └── ethio_bot.urdf               # Three-wheel robot description
│   ├── resource/
│   │   └── ethiopia_navigation
│   ├── package.xml
│   └── setup.py
└── README.md                            # This file
```

---

## Prerequisites

**Questions 1-4** require only:
- Python 3.10+

**Question 5** additionally requires:
- Ubuntu 24.04 (native or WSL2 on Windows)
- ROS 2 Jazzy Jalisco
- Gazebo Sim 8 (Harmonic)
- colcon build tool

---

## Running Questions 1-4

All demos must be run from this directory using `python -m`:

```bash
cd '/mnt/c/Users/HP/Documents/AI/AI principles/traveling_ethiopia/traveling_ethiopia'

# Question 1: BFS and DFS (Figure 1)
python -m uninformed_search.demo

# Question 2: Uniform Cost Search - single goal (Figure 2)
python -m uniform_cost_search.demo

# Question 2: Uniform Cost Search - multi goal (Figure 2)
python -m uniform_cost_search.demo_multi_goal

# Question 3: A* Search (Figure 3)
python -m astar_search.demo

# Question 4: MiniMax adversarial search (Figure 4)
python -m minmax_search.minmax
```

### Expected Output

**Question 1** -- BFS finds the shortest path (fewest hops); DFS finds a valid path (may be longer):

```
BFS Path:
['Addis Ababa', ..., 'Lalibela']

DFS Path:
['Addis Ababa', ..., 'Lalibela']
```

**Question 2** -- UCS finds the lowest-cost path to Lalibela, then the multi-goal variant visits all 8 cities:

```
UCS Path from Addis Ababa to Lalibela:
['Addis Ababa', ..., 'Lalibela']
Total Cost: ...
```

**Question 3** -- A* finds the optimal path from Addis Ababa to Moyale using heuristics:

```
A* Path from Addis Ababa to Moyale:
['Addis Ababa', ..., 'Moyale']
Total Cost: ...
```

**Question 4** -- MiniMax directs the agent to the best coffee destination against an adversary:

```
Starting from: Addis Ababa
Best path: Addis Ababa -> ... -> ...
Final utility (coffee quality): ...
```

---

## Question 5: ROS 2 Robot Navigation

### What It Does

- **5.1**: Three-wheel functional robot with physics engine and sensors (RGB camera, proximity lidar, IMU/gyroscope).
- **5.2**: A `.world` file with all 41 states from Figure 5 in a Cartesian coordinate system.
- **5.3**: A ROS 2-based controller that uses BFS to navigate between any two cities in Figure 5.

### Installation

#### 1. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common curl gnupg lsb-release -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install ros-jazzy-desktop -y
sudo apt install python3-colcon-common-extensions python3-argcomplete -y

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

#### 2. Install Gazebo Sim 8 and ROS-Gazebo Bridge

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install gz-sim8-cli libgz-sim8 libgz-sim8-plugins gz-tools2 -y
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge -y
```

#### 3. (WSL2 only) GPU Permissions

If you see `libEGL warning: failed to open /dev/dri/renderD128` when launching Gazebo:

```bash
sudo usermod -aG render $USER
```

Then restart WSL from PowerShell:

```powershell
wsl --shutdown
```

### Building the ROS 2 Package

```bash
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/HP/Documents/AI/AI principles/traveling_ethiopia/traveling_ethiopia/ros_navigation'

colcon build
source install/setup.bash
```

### Running the Simulation

#### Option A: Full Simulation (Recommended)

Launch Gazebo, the ROS-Gazebo bridges, and the navigation controller all at once:

```bash
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/HP/Documents/AI/AI principles/traveling_ethiopia/traveling_ethiopia/ros_navigation'
source install/setup.bash

ros2 launch ethiopia_navigation ethiopia_navigation.launch.py
```

To specify a custom start and goal city:

```bash
ros2 launch ethiopia_navigation ethiopia_navigation.launch.py start:="Jimma" goal:="Dire Dawa"
```

What happens:
1. Gazebo Sim opens with the Ethiopia world (41 city markers visible).
2. The three-wheel robot spawns at the origin (Addis Ababa by default).
3. ROS-Gazebo topic bridges start for `/cmd_vel` and `/odom`.
4. The controller plans a BFS path and drives the robot city-to-city.
5. Progress is logged to the terminal as each city is reached.

**Important**: After Gazebo opens, click the **Play** button (bottom-left) to start the physics simulation. The robot will not move until the simulation is running.

#### Option B: Manual Multi-Terminal Setup

**Terminal 1** -- Launch Gazebo:

```bash
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/HP/Documents/AI/AI principles/traveling_ethiopia/traveling_ethiopia/ros_navigation'
gz sim install/ethiopia_navigation/share/ethiopia_navigation/worlds/ethiopia.world
```

**Terminal 2** -- Start bridges and controller:

```bash
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/HP/Documents/AI/AI principles/traveling_ethiopia/traveling_ethiopia/ros_navigation'
source install/setup.bash

ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry &

sleep 2

install/ethiopia_navigation/bin/ethiopia_controller --start "Addis Ababa" --goal "Harar"
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `start` | `Addis Ababa` | Starting city (must be a city from Figure 5) |
| `goal` | `Harar` | Goal city (must be a city from Figure 5) |

Examples:

```bash
ros2 launch ethiopia_navigation ethiopia_navigation.launch.py start:="Addis Ababa" goal:="Gode"
ros2 launch ethiopia_navigation ethiopia_navigation.launch.py start:="Hawassa" goal:="Dire Dawa"
ros2 launch ethiopia_navigation ethiopia_navigation.launch.py start:="Jimma" goal:="Dega Habur"
ros2 launch ethiopia_navigation ethiopia_navigation.launch.py start:="Gambella" goal:="Arba Minch"
```

### Available Cities (Figure 5)

All **41 cities** from the relaxed state space graph:

| # | City | # | City | # | City |
|---|------|---|------|---|------|
| 1 | Addis Ababa | 15 | Gore | 29 | Harar |
| 2 | Adama | 16 | Gambella | 30 | Babile |
| 3 | Ambo | 17 | Tepi | 31 | Jijiga |
| 4 | Debre Birhan | 18 | Bonga | 32 | Dega Habur |
| 5 | Wolkite | 19 | Mezan Tefari | 33 | Kebri Dehar |
| 6 | Buta Jirra | 20 | Jimma | 34 | Gode |
| 7 | Worabe | 21 | Hossana | 35 | Assella |
| 8 | Batu | 22 | Shashemene | 36 | Assasa |
| 9 | Nekemte | 23 | Hawassa | 37 | Dodola |
| 10 | Gimbi | 24 | Dilla | 38 | Bale |
| 11 | Dembi Dollo | 25 | Wolaita Sodo | 39 | Goba |
| 12 | Bedelle | 26 | Dawro | 40 | Sof Oumer |
| 13 | Matahara | 27 | Arba Minch | 41 | Chiro |
| 14 | Awash | 28 | Dire Dawa | | |

### Robot Specifications

| Property | Value |
|----------|-------|
| Drive type | Differential drive (2 powered wheels + 1 caster) |
| Physics | SDF 1.8, Gazebo Sim 8 physics engine |
| RGB Camera | Forward-facing, mounted on chassis |
| IMU / Gyroscope | Measures orientation and angular velocity |
| Proximity Lidar | 360-degree range sensor |
| Wheel friction | Drive wheels with friction; caster with ball joint |

---

## Algorithm Details

### BFS (Question 1, Question 5)

- **Complete**: Guaranteed to find a solution if one exists.
- **Optimal**: Finds the shortest path in number of edges.
- **Data structure**: FIFO queue.

### DFS (Question 1)

- **Complete**: Yes (with cycle detection).
- **Not optimal**: May find a longer path than BFS.
- **Data structure**: LIFO stack.

### Uniform Cost Search (Question 2)

- **Complete**: Yes.
- **Optimal**: Finds the lowest-cost path using edge weights.
- **Data structure**: Priority queue (min-heap).
- **Multi-goal variant**: Greedily visits the nearest unvisited goal at each step.

### A* Search (Question 3)

- **Complete**: Yes.
- **Optimal**: With admissible and consistent heuristic.
- **Evaluation function**: f(n) = g(n) + h(n).

### MiniMax (Question 4)

- **Two-player adversarial search**: Agent maximizes utility, adversary minimizes it.
- **Terminal states**: Coffee-producing cities with quality scores.

### Navigation Controller (Question 5)

The ROS 2 controller uses a proportional controller to drive the robot:
1. **Rotate** toward the next city until heading error is small.
2. **Drive forward** while correcting heading.
3. When within 0.3 m of the target coordinates, mark the city as reached and proceed to the next.

---

## Troubleshooting

### ROS 2 not found

```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### colcon build fails

```bash
sudo apt install python3-colcon-common-extensions -y
```

### Gazebo not installed

```bash
sudo apt install gz-sim8-cli libgz-sim8 libgz-sim8-plugins gz-tools2 -y
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge -y
```

### Robot not moving in Gazebo

- Make sure you clicked the **Play** button in Gazebo (bottom-left of the window).
- Check that the ROS-Gazebo bridges are running (look for `parameter_bridge` processes).
- Verify the `/cmd_vel` topic is being published: `ros2 topic echo /cmd_vel`.

### Package not found after build

```bash
colcon build
source install/setup.bash
```

Or run the controller directly:

```bash
install/ethiopia_navigation/bin/ethiopia_controller --start "Addis Ababa" --goal "Harar"
```

### WSL2 GPU warnings

The `libEGL warning` messages are non-fatal. Gazebo falls back to software rendering. To suppress them:

```bash
sudo usermod -aG render $USER
# Then restart WSL from PowerShell: wsl --shutdown
```

### Quick Setup Alias

Add this to your `~/.bashrc` so you can set up the environment in one command:

```bash
alias ethiopia_nav='cd "/mnt/c/Users/HP/Documents/AI/AI principles/traveling_ethiopia/traveling_ethiopia/ros_navigation" && source /opt/ros/jazzy/setup.bash && source install/setup.bash'
```

Then just run:

```bash
ethiopia_nav
ros2 launch ethiopia_navigation ethiopia_navigation.launch.py start:="Addis Ababa" goal:="Harar"
```

---

## License

Apache License 2.0

## Author

Etubdink Zebre
