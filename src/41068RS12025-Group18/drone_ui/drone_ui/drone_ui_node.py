import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int32MultiArray
from PyQt5 import QtWidgets
from drone_ui.drone_ui_code import Ui_MainWindow
from std_msgs.msg import Bool
import threading

#Ive got the row and column publishers here for the tree selection but i havent added anything for them to work yet

class DroneUINode(Node):
    def __init__(self):
        super().__init__('drone_ui_node')

        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        self.tree_data = {}

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

        x_map = {1: -4, 2: 0, 3: 4}      
        y_map = {1: -10, 2: -6, 3: -2, 4: 2, 5: 6, 6: 10}  

        x = x_map[column]
        y = y_map[row]
        
        if (x, y) in self.tree_data:
            width = self.tree_data[(x, y)]
            self.ui.WidthLabel.setText(f"Width: {width}cm") 
        else:
            self.ui.WidthLabel.setText(f"Width: N/A") 

    # --- Subscriber callbacks ---
    def status_callback(self, msg):
        self.ui.StatusLabel.setText(msg.data)

    def width_callback(self, msg):
        data = msg.data
    
        for i in range(0, len(data), 3):
            width = data[i]
            x = data[i + 1]
            y = data[i + 2]
            
            # if width > 0:
            self.tree_data[(x, y)] = width

    def run(self):
        sys.exit(self.app.exec_())

def main():
    rclpy.init()
    node = DroneUINode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
