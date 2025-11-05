# Drone UI - Tree Selection Integration

## Overview
This document describes the integration of the Drone UI with the tree detection system, enabling operators to select and view tree measurements by row and column.

## Integration Status: ✅ COMPLETE

The UI is now fully integrated with the tree detection system and can:
1. Subscribe to tree detection data from the LiDAR sensor
2. Store detected tree information organized by grid coordinates
3. Allow operators to select trees by row and column
4. Display the measured width of selected trees

---

## System Architecture

### Data Flow
```
LiDAR Sensor (/scan)
    ↓
[lidar_tree_detector_node] (C++)
    ↓ publishes
/known_tree_widths (Int32MultiArray)
    ↓ subscribes
[drone_ui_node] (Python/PyQt5)
    ↓
User Interface Display
```

---

## Tree Grid Layout

The plantation consists of **18 trees** arranged in a 3x6 grid:

### World Coordinates
- **Columns (X-axis):** -4m, 0m, 4m (3 columns)
- **Rows (Y-axis):** -10m, -6m, -2m, 2m, 6m, 10m (6 rows)

### UI Mapping
| UI Column | X Coordinate | UI Row | Y Coordinate |
|-----------|-------------|--------|-------------|
| 1 | -4m | 1 | -10m |
| 2 | 0m | 2 | -6m |
| 3 | 4m | 3 | -2m |
| | | 4 | 2m |
| | | 5 | 6m |
| | | 6 | 10m |

### Visual Layout
```
        Column 1    Column 2    Column 3
         (X=-4)      (X=0)       (X=4)
Row 1    Tree        Tree        Tree      (Y=-10)
Row 2    Tree        Tree        Tree      (Y=-6)
Row 3    Tree        Tree        Tree      (Y=-2)
Row 4    Tree        Tree        Tree      (Y=2)
Row 5    Tree        Tree        Tree      (Y=6)
Row 6    Tree        Tree        Tree      (Y=10)
```

---

## ROS2 Topics

### Subscribed Topics
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/known_tree_widths` | `Int32MultiArray` | Tree detection data from LiDAR |
| `/drone/status` | `String` | Mission status updates |

### Published Topics
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/drone/cmd/start` | `Bool` | Start mission command |
| `/drone/cmd/stop` | `Bool` | Stop mission command |
| `/drone/cmd/return_home` | `Bool` | Return home command |
| `/drone/cmd/height` | `Float32` | Flight height setting |
| `/drone/cmd/tree_row_select` | `Float32` | Selected tree row |
| `/drone/cmd/tree_column_select` | `Float32` | Selected tree column |

---

## Data Format

### /known_tree_widths Message Structure
The `Int32MultiArray` contains tree data in groups of 3 values:
```
[width_cm, x_int, y_int, width_cm, x_int, y_int, ...]
```

**Example:**
```
[25, -4, -10, 30, 0, -10, 28, 4, -10, ...]
```
This represents:
- Tree at (-4, -10): 25 cm width
- Tree at (0, -10): 30 cm width
- Tree at (4, -10): 28 cm width

---

## UI Components

### Control Panel
1. **Start Drone** - Initiates autonomous mission
2. **Stop Drone** - Halts current mission
3. **Return Home** - Returns drone to home base
4. **Flight Height** - Sets drone altitude (0.25-5.0m)

### Tree Selection Interface
1. **Row SpinBox (R:)** - Select tree row (1-6)
2. **Column SpinBox (C:)** - Select tree column (1-3)
3. **Get Width Button** - Retrieve width for selected tree
4. **Width Label** - Displays tree width in cm or "N/A"

### Status Display
1. **Status Label** - Shows mission status (IDLE/RUNNING/RETURNING HOME)

---

## Implementation Details

### Thread Safety
The UI uses separate threads for ROS2 and Qt operations:
- **ROS Thread:** Handles topic subscriptions and data callbacks
- **Main Thread:** Runs Qt event loop and UI updates
- **Thread Lock:** Ensures safe access to shared `tree_data` dictionary

### Code Location
**File:** `/home/user/Personal/41068RS12025-Group18/drone_ui/drone_ui/drone_ui_node.py`

### Key Functions

#### `width_callback(msg)`
```python
def width_callback(self, msg):
    """Receives tree detection data from lidar_tree_detector_node"""
    # Parses Int32MultiArray: [width, x, y, ...]
    # Updates tree_data dictionary with (x,y): width mapping
```

#### `get_width()`
```python
def get_width(self):
    """Retrieves and displays width for selected tree"""
    # Maps UI row/column to world coordinates
    # Looks up tree in tree_data dictionary
    # Updates WidthLabel with result or "N/A"
```

---

## Usage Instructions

### 1. Launch the System

#### Terminal 1: Launch Simulation
```bash
cd ~/Personal/41068RS12025-Group18
source install/setup.bash
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py
```

#### Terminal 2: Launch Tree Detection
```bash
cd ~/Personal/41068RS12025-Group18
source install/setup.bash
ros2 run lidar_tree_detector lidar_tree_detector_node
```

#### Terminal 3: Launch UI
```bash
cd ~/Personal/41068RS12025-Group18
source install/setup.bash
ros2 run drone_ui drone_ui_node
```

### 2. Select a Tree
1. Use the **Row** spinbox to select a row (1-6)
2. Use the **Column** spinbox to select a column (1-3)
3. Click **Get Width** button
4. The width will be displayed in the **Width Label**

### 3. Interpreting Results
- **"Width: XX cm"** - Tree detected with measured width
- **"Width: N/A"** - No data available for that tree location

---

## Testing & Validation

### Verification Checklist
- [x] UI subscribes to `/known_tree_widths` topic
- [x] Tree data is parsed correctly from Int32MultiArray
- [x] Row/Column selectors map to correct world coordinates
- [x] Width display updates when "Get Width" is clicked
- [x] Thread-safe access to shared data
- [x] Proper error handling for missing data
- [x] Logging for debugging and monitoring

### Expected Behavior
1. **On Launch:** UI opens with default settings
2. **During Mission:** Tree data updates as drone detects trees
3. **On Selection:** Width displays immediately if data available
4. **On Invalid Selection:** "N/A" displayed with warning log

---

## Troubleshooting

### Issue: "Width: N/A" for all trees
**Possible Causes:**
1. Tree detector node not running
2. Drone too far from trees for LiDAR detection
3. Tree not yet scanned by drone

**Solutions:**
```bash
# Check if tree detector is running
ros2 node list | grep lidar_tree_detector

# Check if data is being published
ros2 topic echo /known_tree_widths

# Check if UI is receiving data (check logs)
ros2 node info /drone_ui_node
```

### Issue: UI not responding
**Solution:**
```bash
# Restart the UI node
ros2 run drone_ui drone_ui_node
```

### Issue: Incorrect tree coordinates
**Verification:**
Check the mapping in `get_width()` function matches the physical tree layout in the simulation world.

---

## Future Enhancements

### Potential Improvements
1. **Visual Map Display** - Show all 18 trees with color-coded status
2. **Real-time Updates** - Auto-refresh width when new data arrives
3. **Tree Statistics** - Display average width, detection confidence
4. **Export Functionality** - Save tree inventory to CSV/JSON
5. **3D Visualization** - Integrate with RViz markers

---

## Technical Specifications

### Dependencies
- **ROS2:** Humble (or compatible)
- **Python:** 3.10+
- **PyQt5:** 5.15+
- **rclpy:** ROS2 Python client library
- **std_msgs:** Standard ROS message types

### Performance
- **Update Rate:** Real-time as data arrives
- **UI Responsiveness:** Non-blocking (separate ROS thread)
- **Memory:** O(n) where n = number of detected trees (max 18)

---

## Code Changes Summary

### Modified Files
1. **drone_ui_node.py**
   - Added thread safety with `threading.Lock()`
   - Enhanced logging for debugging
   - Added comprehensive documentation
   - Improved error handling
   - Fixed duplicate imports and methods

### Improvements Made
1. ✅ Thread-safe data access
2. ✅ Better logging and debugging
3. ✅ Input validation and error handling
4. ✅ Comprehensive code documentation
5. ✅ Clear coordinate mapping comments

---

## Testing Recommendations

### Unit Tests
```python
# Test coordinate mapping
assert x_map[1] == -4
assert y_map[6] == 10

# Test data parsing
test_data = [25, -4, -10, 30, 0, -10]
# Verify correct parsing into tree_data
```

### Integration Tests
1. Launch all nodes
2. Verify topic connections
3. Test tree selection for all 18 positions
4. Verify width display updates correctly

### System Tests
1. Full mission run
2. Verify all trees detected
3. Confirm UI displays correct widths
4. Test edge cases (no data, invalid selection)

---

## Contact & Support

**Package Maintainer:** benjamin.j.costarella@student.uts.edu.au

**Package Location:** `/home/user/Personal/41068RS12025-Group18/drone_ui/`

**Documentation:** See inline code comments in `drone_ui_node.py`

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2025-11-05 | 1.0 | Initial integration complete |
| | | - Added thread safety |
| | | - Enhanced logging |
| | | - Added documentation |
| | | - Fixed code issues |

---

## License
TODO: License declaration (see package.xml)
