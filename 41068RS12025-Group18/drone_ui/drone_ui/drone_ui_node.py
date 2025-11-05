#!/usr/bin/env python3
"""
Drone UI Node - PyQt5-based user interface for drone control and tree inventory monitoring

This ROS2 node provides a graphical user interface for:
1. Controlling drone mission (Start, Stop, Return Home)
2. Setting flight parameters (Height control)
3. Viewing detected tree information by selecting row/column
4. Monitoring mission status

Integration with Tree Detection System:
-------------------------------------
The UI subscribes to /known_tree_widths topic (Int32MultiArray) published by the
lidar_tree_detector_node. The data format is:
    [width_cm, x_int, y_int, ...] in groups of 3

Tree Grid Layout (18 trees total):
    - 6 rows (Y coordinates): -10, -6, -2, 2, 6, 10 meters
    - 3 columns (X coordinates): -4, 0, 4 meters
    - UI Row selector: 1-6 maps to Y: -10 to 10
    - UI Column selector: 1-3 maps to X: -4, 0, 4

The tree_data dictionary stores detected trees by (x, y) coordinates,
and the UI allows users to query specific trees by row/column selection.

Topics:
-------
Publishers:
    - /drone/cmd/start (Bool)
    - /drone/cmd/stop (Bool)
    - /drone/cmd/return_home (Bool)
    - /drone/cmd/height (Float32)
    - /drone/cmd/tree_row_select (Float32)
    - /drone/cmd/tree_column_select (Float32)

Subscribers:
    - /drone/status (String) - Updates status display
    - /known_tree_widths (Int32MultiArray) - Tree detection data from LiDAR
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int32MultiArray
from PyQt5 import QtWidgets
from drone_ui.drone_ui_code import Ui_MainWindow
import threading

class DroneUINode(Node):
    def __init__(self):
        super().__init__('drone_ui_node')

        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        # Thread-safe storage for tree detection data
        self.tree_data = {}
        self.tree_data_lock = threading.Lock()

        # Publishers
        self.start_pub = self.create_publisher(Bool, '/drone/cmd/start', 10)
        self.stop_pub = self.create_publisher(Bool, '/drone/cmd/stop', 10)
        self.home_pub = self.create_publisher(Bool, '/drone/cmd/return_home', 10)
        self.height_pub = self.create_publisher(Float32, '/drone/cmd/height', 10)
        self.tree_row_pub = self.create_publisher(Float32, '/drone/cmd/tree_row_select', 10)
        self.tree_column_pub = self.create_publisher(Float32, '/drone/cmd/tree_column_select', 10)

        # Subscribers
        self.create_subscription(String, '/drone/status', self.status_callback, 10)
        self.create_subscription(Int32MultiArray, '/known_tree_widths', self.width_callback, 10) #This will be changed depending on the LIDAR topic

        # Setup Qt
        self.app = QtWidgets.QApplication(sys.argv)
        self.window = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.window)

        # Connect UI buttons to ROS functions
        self.ui.Start.clicked.connect(self.start_drone)
        self.ui.Stop.clicked.connect(self.stop_drone)
        self.ui.ReturnHome.clicked.connect(self.return_home)
        self.ui.HeightControl.valueChanged.connect(self.set_height)
        self.ui.GetWidth.clicked.connect(self.get_width)

        self.window.show()


    def spin_ros(self):
        rclpy.spin(self)

    def run(self):
        sys.exit(self.app.exec_())


    # --- Publisher callbacks ---
    def start_drone(self):
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)
        self.get_logger().info("Start command sent")

    def stop_drone(self):
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)
        self.get_logger().info("Stop command sent")

    def return_home(self):
        msg = Bool()
        msg.data = True
        self.home_pub.publish(msg)
        self.get_logger().info("Return Home command sent")

    def set_height(self, value):
        msg = Float32()
        msg.data = float(value)
        self.height_pub.publish(msg)
        self.get_logger().info(f"Height set to {value:.2f} m")

    def get_width(self):
        row = int(self.ui.TreeRow.value())
        column = int(self.ui.TreeColumn.value())

        # Map UI row/column to world coordinates
        # Columns: 1=X:-4m, 2=X:0m, 3=X:4m
        # Rows: 1=Y:-10m, 2=Y:-6m, 3=Y:-2m, 4=Y:2m, 5=Y:6m, 6=Y:10m
        x_map = {1: -4, 2: 0, 3: 4}
        y_map = {1: -10, 2: -6, 3: -2, 4: 2, 5: 6, 6: 10}

        x = x_map[column]
        y = y_map[row]

        # Thread-safe access to tree data
        with self.tree_data_lock:
            if (x, y) in self.tree_data:
                width = self.tree_data[(x, y)]
                self.ui.WidthLabel.setText(f"Width: {width} cm")
                self.get_logger().info(f"Tree at Row {row}, Column {column} (x={x}, y={y}): {width} cm")
            else:
                self.ui.WidthLabel.setText(f"Width: N/A")
                self.get_logger().warn(f"No data for tree at Row {row}, Column {column} (x={x}, y={y})") 

    # --- Subscriber callbacks ---
    def status_callback(self, msg):
        self.ui.StatusLabel.setText(msg.data)

    def width_callback(self, msg):
        """Callback for receiving tree detection data from lidar_tree_detector.
        Data format: [width_cm, x_int, y_int, ...] in groups of 3
        """
        data = msg.data

        if len(data) % 3 != 0:
            self.get_logger().error(f"Invalid tree data received: expected multiples of 3, got {len(data)}")
            return

        # Thread-safe update of tree data
        with self.tree_data_lock:
            trees_updated = 0
            for i in range(0, len(data), 3):
                width = data[i]  # Width in centimeters
                x = data[i + 1]  # X coordinate (integer)
                y = data[i + 2]  # Y coordinate (integer)

                # Store or update tree data
                if (x, y) not in self.tree_data or self.tree_data[(x, y)] != width:
                    self.tree_data[(x, y)] = width
                    trees_updated += 1

            if trees_updated > 0:
                self.get_logger().info(f"Updated {trees_updated} tree(s). Total trees detected: {len(self.tree_data)}")

def main():
    rclpy.init()
    node = DroneUINode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
