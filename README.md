# Fielder_gz — ROS2 Gazebo Simulator

A ROS2-based robot simulation project using Gazebo Harmonic, Nav2, and SLAM Toolbox for autonomous navigation and mapping.

---

## Prerequisites

- **OS:** Ubuntu 24.04 (Noble)
- **ROS2 Jazzy** fully installed — follow the [official ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation.html) before continuing

Verify your ROS2 installation before proceeding:

```bash
echo $ROS_DISTRO
# Expected output: jazzy
```

---

## 1. Install Package Dependencies

```bash
sudo apt update

# URDF/Xacro, joint state publisher, SLAM, Nav2, and Gazebo-ROS bridge
sudo apt install \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge
```
# Gazebo Harmonic (standalone simulator)

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```
Confirm Gazebo installed correctly:

```bash
gz sim --version
```

> **Tip:** If `gz sim --version` fails, source ROS2 first and retry:
> ```bash
> source /opt/ros/jazzy/setup.bash && gz sim --version
> ```

---

## 2. Full Workflow — Clone to Running Simulation

### Step 1 — Create the Workspace and Clone the Repository

```bash
# Create the workspace directory
mkdir -p ~/ros2_ws/src

# Navigate into src and clone the repository
cd ~/ros2_ws/src
git clone https://github.com/zhlmi25/fielder_gz.git
```

Your workspace should now look like this:

```
~/ros2_ws/
└── src/
    └── fielder_gz/
```

### Step 2 — Resolve Package Dependencies

From the **workspace root**, run rosdep to install any remaining dependencies declared in `package.xml`:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

> **What each flag does:**
> - `--from-paths src` — scan the `src/` folder for packages
> - `--ignore-src` — skip packages already present as source code
> - `-r` — continue even if some optional dependencies are unavailable
> - `-y` — automatically confirm all install prompts

### Step 3 — Build the Workspace

Always build from the **workspace root** (`~/ros2_ws`), not from inside `src/`.

```bash
cd ~/ros2_ws
colcon build

# Or build only fielder_robot (faster during development)
colcon build --packages-select fielder_robot
```

> **Tip:** Use `--symlink-install` to avoid rebuilding after editing Python scripts or launch files:
> ```bash
> colcon build --symlink-install
> ```

After a successful build your workspace will contain:

```
~/ros2_ws/
├── src/        ← your source code
├── build/      ← intermediate build files (auto-generated)
├── install/    ← compiled outputs (auto-generated)
└── log/        ← build logs (auto-generated)
```

### Step 4 — Source the Workspace

After building, source the workspace so ROS2 can find the `fielder_robot` package:

```bash
source ~/ros2_ws/install/setup.bash
```

Add it permanently to `~/.bashrc` so every new terminal is ready:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Launch Files

### Visualize the Robot in RViz2

Launches RViz2 with the robot URDF, robot description, and joint states loaded.

```bash
ros2 launch fielder_gz display.launch.py
```

### Run the Gazebo Simulation

Spawns the robot in a preloaded Gazebo world with all required plugins active.

```bash
ros2 launch fielder_gz fielder_sim.launch.py
```

### Create a Map (SLAM Mapping Mode)

Runs SLAM Toolbox to build a map of the environment. Drive the robot around to explore, then save the map.

```bash
ros2 launch fielder_gz mapping.launch.py
```

> **Tip:** Maps are saved to the `maps/` directory. Save the map before closing SLAM Toolbox or your progress will be lost.

### Localize the Robot

Runs SLAM Toolbox in localization mode using a previously saved map. **Must be launched before navigation.**

```bash
ros2 launch fielder_gz localization.launch.py
```

### Autonomous Navigation (Nav2)

Runs the Nav2 stack for autonomous navigation. Requires localization to already be running.

```bash
# Terminal 1 — start localization first
ros2 launch fielder_gz localization.launch.py

# Terminal 2 — then start navigation
ros2 launch fielder_gz navigation.launch.py
```

### Full Bringup (All-in-One)

Launches the complete stack — Gazebo, SLAM, localization, and Nav2 — in a single command.

```bash
ros2 launch fielder_gz fielder_bringup.launch.py
```

---

## 4. Project Structure

```
fielder_gz/
├── config/                         # Configuration files for each launch file
│                                   # Edit these to tune simulation, SLAM, and Nav2 parameters
├── launch/
│   ├── display.launch.py           # RViz2 visualization
│   ├── fielder_sim.launch.py       # Gazebo simulation with world + plugins
│   ├── mapping.launch.py           # SLAM Toolbox — map creation
│   ├── localization.launch.py      # SLAM Toolbox — localization only
│   ├── navigation.launch.py        # Nav2 autonomous navigation
│   └── fielder_bringup.launch.py   # Full stack bringup (all-in-one)
├── maps/                           # Stores maps generated by the mapping process
├── urdf/
│   ├── fielder_chassis.urdf.xacro  # Robot chassis geometry
│   ├── gazebo_control.xacro        # Odometry plugin for simulation
│   ├── gazebo_fielder.xacro        # Root Xacro — includes all URDF files
│   ├── gazebo_lidar.xacro          # LIDAR sensor simulation plugin
│   └── gazebo_visual.xacro         # Visual appearance, colors, and materials
├── worlds/                         # Preloaded Gazebo world(s) — add custom .sdf files here
├── CMakeLists.txt                  # Build instructions
└── package.xml                     # Package metadata and dependencies
```

---

## 5. Configuration

All tunable parameters live in the `config/` directory. Edit these to adjust behaviour without touching launch files directly.

| Launch File | Config Purpose |
|---|---|
| `display.launch.py` | RViz2 display config |
| `fielder_sim.launch.py` | Gazebo world and plugin settings |
| `mapping.launch.py` | SLAM Toolbox mapping parameters |
| `localization.launch.py` | SLAM Toolbox localization parameters |
| `navigation.launch.py` | Nav2 planner, controller, and costmap settings |

---

## 6. Verify Everything is Running

```bash
# List all active nodes
ros2 node list

# List all active topics
ros2 topic list

# Check the TF tree (useful for debugging transform issues)
ros2 run tf2_tools view_frames

# Inspect LIDAR data
ros2 topic echo /scan

# Inspect odometry
ros2 topic echo /odom
```

---

## Troubleshooting

**Build fails with missing dependencies**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

**`gz sim` not found or crashes on launch**
```bash
source /opt/ros/jazzy/setup.bash
gz sim --version
```

**Package not found after building**
```bash
source ~/ros2_ws/install/setup.bash
```

**Old build artifacts causing strange errors**
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build
```

**URDF or Xacro errors in Gazebo**
```bash
check_urdf <(xacro src/fielder_robot/urdf/gazebo_fielder.urdf.xacro)
```

**Navigation not working** — localization must be running *before* navigation:
```bash
# Terminal 1
ros2 launch fielder_gz localization.launch.py
# Terminal 2
ros2 launch fielder_gz navigation.launch.py
```

---

## Additional Resources

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [colcon Build Documentation](https://colcon.readthedocs.io/en/released/)

