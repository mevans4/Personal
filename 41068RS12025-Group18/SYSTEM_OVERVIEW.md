# Forest Drone Inventory System - Technical Overview

## System Architecture

This is a comprehensive ROS2-based autonomous drone system designed for forest plantation inventory management. The system autonomously navigates a simulated pine plantation, detects trees using LIDAR, measures tree trunk diameters, and provides a user interface for mission control and data visualization.

## Key Components

### 1. Perception & Mapping
**Package:** `lidar_tree_detector`
**Node:** `lidar_tree_detector_node.cpp`

**Features:**
- **LIDAR-based tree detection** using DBSCAN-like clustering algorithm
- **RANSAC circle fitting** for accurate tree diameter calculation (DBH - Diameter at Breast Height)
- Known tree location matching (18 trees in 3 rows of 6)
- Real-time visualization markers for RViz
- CSV data export for tree measurements
- Publishes to `/known_tree_widths` topic (Int32MultiArray format: [width_cm, x, y, ...])

**Key Parameters:**
- `MAX_RANGE`: 15.0m - Maximum LIDAR range for tree detection
- `CLUSTER_EPS`: 0.6m - Clustering distance threshold
- `CLUSTER_MIN`: 3 points - Minimum cluster size
- `MATCH_RADIUS`: 1.0m - Tree location matching radius
- RANSAC iterations: 100
- RANSAC inlier threshold: 5cm

**Recent Improvements:**
✅ **Implemented RANSAC circle fitting** for accurate diameter calculation (replacing simple max-distance method)
✅ Added sanity checks for reasonable tree trunk sizes (5cm - 1m radius)
✅ Fallback mechanism for edge cases

### 2. Mission Control & Navigation
**Package:** `drone_controller`
**Node:** `mission_node.cpp`

**Features:**
- State machine for mission lifecycle (IDLE → TAKEOFF → FLYING → RETURNING HOME)
- Integration with Nav2 for autonomous waypoint navigation
- UI command integration (start, stop, return home)
- Status publishing for UI feedback
- Altitude control during takeoff

**Mission Pattern:**
The drone follows a systematic plantation survey pattern:
1. **Takeoff** to configured height (default 0.5m)
2. **Row 1** (X=0): Fly from Y=-12 to Y=12 (covering 6 trees)
3. **Row 2** (X=-4): Fly from Y=12 to Y=-12 (covering 6 trees)
4. **Row 3** (X=4): Fly from Y=-12 to Y=12 (covering 6 trees)
5. **Return Home** to base position (-2.0, -12.0)

**Recent Improvements:**
✅ **Updated mission waypoints** to properly align with plantation layout
✅ **Fixed home base coordinates** to match spawn location
✅ Added descriptive comments for mission waypoints

### 3. User Interface
**Package:** `drone_ui`
**Node:** `drone_ui_node.py`

**Features:**
- PyQt5-based graphical interface
- Real-time mission status display
- Mission control buttons (Start, Stop, Return Home)
- Flight height configuration
- Tree selection by row/column with width display
- Integration with tree detection data

**UI Layout:**
- **Inputs:**
  - Start/Stop/Return Home buttons
  - Flight Height spin box
  - Tree Row/Column selectors (6 rows × 3 columns)
  - Get Width button

- **Outputs:**
  - Status label (IDLE/TAKING OFF/FLYING MISSION/RETURNING HOME)
  - Tree width display in centimeters

**Topic Integration:**
- Publishes: `/drone/cmd/start`, `/drone/cmd/stop`, `/drone/cmd/return_home`, `/drone/cmd/height`
- Subscribes: `/drone/status`, `/known_tree_widths`

### 4. Simulation Environment
**Package:** `41068_ignition_bringup`

**World Configuration:**
- **World File:** `Plantation2.sdf`
- **Layout:** 18 pine trees arranged in 3 rows × 6 columns
  - Row 1 (X=0): Y positions [-10, -6, -2, 2, 6, 10]
  - Row 2 (X=-4): Y positions [-10, -6, -2, 2, 6, 10]
  - Row 3 (X=4): Y positions [-10, -6, -2, 2, 6, 10]
- **Drone Spawn:** Position (-2.0, -12.0, 0.5), Yaw: 90° (facing north)

**Launch Files:**
- `41068_ignition_drone.launch.py` - Main simulation launch
- `41068_navigation.launch.py` - Nav2 navigation stack

## System Integration

### Data Flow
```
LIDAR Sensor → /scan topic
    ↓
Tree Detector Node
    ↓ (clustering + RANSAC)
    ↓
/known_tree_widths topic
    ↓
UI Node (displays tree data)
```

```
UI (Start Button) → /drone/cmd/start
    ↓
Mission Control Node
    ↓ (sends waypoints)
    ↓
Nav2 Navigation Stack
    ↓
Drone Movement
```

### Key Topics
- `/scan` (LaserScan) - LIDAR data
- `/odometry` (Odometry) - Drone position
- `/known_tree_widths` (Int32MultiArray) - Detected tree data
- `/drone/status` (String) - Mission status
- `/drone/cmd/*` (Bool/Float32) - UI commands
- `/cmd_vel` (Twist) - Velocity commands

## Requirements Status

| Requirement | Status | Implementation |
|------------|--------|----------------|
| R1: Stable flight at target altitude | ✅ MVP | Altitude controller in mission_node |
| R2: Autonomous navigation | ✅ MVP | Nav2 integration with waypoint navigation |
| R3: Detect tree trunks using LIDAR | ✅ MVP | DBSCAN clustering + RANSAC fitting |
| R5: Centralized visualization | ⚠️ Stretch | RViz markers implemented |
| R6: User-initiated mission | ✅ MVP | UI with Start/Stop/Return buttons |
| R7: Tree diameter calculation | ✅ MVP | RANSAC circle fitting (NEW) |
| R8: Collision avoidance | ⚠️ Stretch | Nav2 costmap-based avoidance |
| R10: Return to home base | ✅ MVP | Return home navigation |

## Technical Specifications

### RANSAC Circle Fitting Algorithm
The diameter calculation uses RANSAC (Random Sample Consensus) for robust circle fitting:

1. **Sampling:** Randomly select 3 points from LIDAR cluster
2. **Model Fitting:** Fit circle through 3 points using determinant method
3. **Validation:** Check for collinearity and reasonable radius (0.025-1.0m)
4. **Inlier Counting:** Count points within 5cm of fitted circle
5. **Best Model Selection:** Keep circle with most inliers
6. **Diameter Extraction:** Return 2 × radius as diameter

**Advantages over max-distance method:**
- More accurate for partial circle scans
- Robust to outliers
- Better handles noise in LIDAR data
- Provides true diameter rather than chord length

### Nav2 Configuration Highlights
- **Planner:** Theta* planner for smooth paths
- **Controller:** DWB local planner with omnidirectional motion model
- **Costmap:** 2D costmap with obstacle layer from LIDAR
- **Robot Radius:** 0.20m for collision avoidance
- **Inflation Radius:** 1.0m for safe tree clearance

## Build Instructions

```bash
# Navigate to workspace
cd /home/user/Personal/41068RS12025-Group18

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Launch Instructions

### Full System Launch
```bash
# Terminal 1: Launch simulation with Plantation2 world
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py world:=Plantation2 rviz:=True nav2:=True

# Terminal 2: Launch tree detector
ros2 run lidar_tree_detector lidar_tree_detector_node

# Terminal 3: Launch mission control
ros2 run drone_controller mission_node

# Terminal 4: Launch UI
ros2 run drone_ui drone_ui_node
```

### Mission Execution
1. Wait for all nodes to initialize
2. In the UI, set desired flight height (default 0.5m recommended)
3. Click "Start" to begin autonomous mission
4. Monitor status in UI and RViz
5. View tree measurements by selecting row/column and clicking "Get Width"
6. Use "Stop" to pause mission, "Return Home" to abort and return

## Data Output

### Tree Detection CSV
Location: `detected_trees.csv` (in current working directory)

Format:
```csv
id,width,center_x,center_y,samples
1,45,-10,0,127
2,38,-6,0,98
...
```

- `id`: Tree ID (1-18)
- `width`: Diameter in centimeters
- `center_x`, `center_y`: Tree center coordinates
- `samples`: Number of LIDAR measurements averaged

## Development Notes

### Recent Changes (Latest Session)
1. ✅ Implemented RANSAC circle fitting for tree diameter calculation
2. ✅ Updated mission waypoints for proper plantation row navigation
3. ✅ Fixed home base coordinates to match spawn location
4. ✅ Added comprehensive code documentation

### Known Limitations
- Nav2 parameters may need tuning for optimal drone navigation
- SLAM localization may drift in repetitive plantation environment
- Tree detection assumes trees are roughly circular at scan height

### Future Enhancements (Stretch Goals)
- [ ] Adaptive LIDAR range limits to prevent analyzing distant trees
- [ ] Multi-pass averaging for improved diameter accuracy
- [ ] Visualization of ready-to-harvest trees (colored markers based on size threshold)
- [ ] Dynamic obstacle avoidance for unexpected objects
- [ ] Flight path optimization for efficiency

## Troubleshooting

### Tree Detection Issues
- Check LIDAR topic: `ros2 topic echo /scan`
- Verify odometry: `ros2 topic echo /odometry`
- Check RViz markers visualization
- Review `detected_trees.csv` for measurement data

### Navigation Issues
- Verify map is loaded: `ros2 topic echo /map`
- Check Nav2 costmaps in RViz
- Monitor `/drone/status` topic for mission state
- Ensure Nav2 action servers are running

### UI Issues
- Check topic connections: `ros2 topic list`
- Verify `/known_tree_widths` is publishing: `ros2 topic echo /known_tree_widths`
- Check ROS node list: `ros2 node list`

## Contributors & Context

This system was developed for COMP41068 Robotics Studio 1 (2025) - Group 18.
The project demonstrates autonomous drone navigation, LIDAR-based perception, and ROS2 integration for real-world forestry applications.

---

**Last Updated:** 2025-11-05
**System Version:** 1.0 (MVP Complete)
