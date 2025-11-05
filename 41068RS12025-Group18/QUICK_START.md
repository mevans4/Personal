# Forest Drone Inventory System - Quick Start Guide

## Prerequisites
- ROS2 (Humble or later)
- Gazebo Ignition
- Nav2 navigation stack
- PyQt5 for Python

## Quick Build & Launch

### 1. Build the Workspace
```bash
cd /home/user/Personal/41068RS12025-Group18
colcon build
source install/setup.bash
```

### 2. Launch the System (One Terminal)
```bash
# Launch everything except UI
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py world:=Plantation2 rviz:=True nav2:=True &
sleep 5
ros2 run lidar_tree_detector lidar_tree_detector_node &
ros2 run drone_controller mission_node &
ros2 run drone_ui drone_ui_node
```

### 3. Run the Mission
1. **Wait** for Gazebo and RViz to load (~10 seconds)
2. In the UI window:
   - Set flight height: 0.5m (default is fine)
   - Click **"Start"** to begin autonomous mission
3. **Watch** in RViz as the drone:
   - Takes off to target height
   - Flies down Row 1 (center, X=0)
   - Moves to Row 2 (left, X=-4) and flies back
   - Moves to Row 3 (right, X=4) and flies down
   - Returns to home base
4. **View tree data:**
   - Select Row (1-6) and Column (1-3) in UI
   - Click **"Get Width"** to see tree diameter in cm

### 4. Check Results
```bash
# View detected trees CSV
cat detected_trees.csv

# Monitor tree detection in real-time
ros2 topic echo /known_tree_widths

# Check mission status
ros2 topic echo /drone/status
```

## Plantation Layout Reference

```
         Column 1    Column 2    Column 3
           (X=-4)      (X=0)       (X=4)
Row 1 (Y=-10)  🌲         🌲          🌲
Row 2 (Y=-6)   🌲         🌲          🌲
Row 3 (Y=-2)   🌲         🌲          🌲
Row 4 (Y=2)    🌲         🌲          🌲
Row 5 (Y=6)    🌲         🌲          🌲
Row 6 (Y=10)   🌲         🌲          🌲

Home Base: (-2, -12) ✈️
```

## Mission Flight Path

```
Start: Home Base (-2, -12)
  ↓ (takeoff to 0.5m)
  ↓
Waypoint 1: (0, -12) ← Start of Row 1
  ↓ (fly north, measuring trees)
Waypoint 2: (0, 12) ← End of Row 1
  ↓ (move west)
Waypoint 3: (-4, 12) ← Start of Row 2
  ↓ (fly south, measuring trees)
Waypoint 4: (-4, -12) ← End of Row 2
  ↓ (move east)
Waypoint 5: (4, -12) ← Start of Row 3
  ↓ (fly north, measuring trees)
Waypoint 6: (4, 12) ← End of Row 3
  ↓ (return home)
End: Home Base (-2, -12) ← Land
```

## UI Controls Quick Reference

| Control | Purpose |
|---------|---------|
| **Start** | Begin autonomous plantation survey mission |
| **Stop** | Pause mission (can resume from last waypoint) |
| **Return Home** | Abort mission and fly back to base |
| **Height Control** | Set flight altitude (0.1 - 5.0m) |
| **Tree Row** | Select tree row (1-6) for data query |
| **Tree Column** | Select tree column (1-3) for data query |
| **Get Width** | Display diameter of selected tree |

## Status Indicators

The UI Status Label shows current mission state:
- `IDLE` - System ready, waiting for start command
- `TAKING OFF` - Ascending to target altitude
- `TAKEOFF COMPLETE` - Reached target height
- `FLYING MISSION` - Executing plantation survey
- `MISSION COMPLETE` - All waypoints reached
- `RETURNING HOME` - Flying back to base
- `STOPPED` - Mission paused by user

## Key Features (Latest Update)

### ✅ RANSAC Circle Fitting
- Accurate tree diameter measurement using RANSAC algorithm
- Robust to LIDAR noise and partial scans
- Sanity checking for realistic tree sizes (5cm - 1m radius)

### ✅ Systematic Row Navigation
- Efficient serpentine pattern covers all trees
- Waypoints positioned for optimal LIDAR coverage
- Automatic return to home base after completion

### ✅ Real-time Data Collection
- Continuous tree detection during flight
- Averaged measurements for improved accuracy
- CSV export for data analysis

## Troubleshooting

### Drone doesn't take off
- Check: Is gravity enabled in world file?
- Check: Is altitude controller publishing to `/cmd_vel`?
- Solution: Monitor `ros2 topic echo /odometry` for position.z

### Trees not detected
- Check: `ros2 topic echo /scan` - Is LIDAR working?
- Check: `ros2 topic list | grep tree` - Is detector running?
- Check: View RViz markers - Are trees being visualized?

### Navigation not working
- Check: Nav2 stack launched? Look for "Nav2 is ready" message
- Check: Map loaded? `ros2 topic echo /map`
- Check: Costmap visible in RViz?

### UI shows "No data for tree"
- Wait: Trees are detected during flight, not before
- Check: Has drone flown past that tree yet?
- Check: `ros2 topic echo /known_tree_widths` for published data

## Expected Results

After a successful mission:
- **18 trees detected** (all trees in plantation)
- **CSV file created** with diameter measurements
- **90%+ accuracy** on tree diameter (within 10% of actual size)
- **All waypoints reached** without collisions
- **Drone returned home** successfully

## Next Steps

1. Review `detected_trees.csv` for measurement data
2. Check RViz visualization for tree markers
3. Analyze measurement accuracy against known tree sizes
4. Experiment with flight height for optimal detection
5. Test stop/resume functionality during mission

## For Developers

- **Tree Detector:** `/41068RS12025-Group18/lidar_tree_detector/src/lidar_tree_detector_node.cpp`
- **Mission Control:** `/41068RS12025-Group18/drone_controller/src/mission_node.cpp`
- **UI:** `/41068RS12025-Group18/drone_ui/drone_ui/drone_ui_node.py`
- **World:** `/41068RS12025-Group18/41068_ignition_bringup/worlds/Plantation2.sdf`

See `SYSTEM_OVERVIEW.md` for detailed technical documentation.

---

**Version:** 1.0 MVP
**Last Updated:** 2025-11-05
