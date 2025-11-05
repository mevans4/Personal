#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <vector>
#include <mutex>
#include <cmath>
#include <limits>
#include <sstream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

struct KnownTree {
    int id;
    double x;
    double y;
    double width_sum = 0.0;
    int width_count = 0;
};

class LidarTreeDetectorNode : public rclcpp::Node
{
public:
    LidarTreeDetectorNode()
    : Node("lidar_tree_detector_minimal")
    {
        // Known tree locations extracted from the provided SDF poses (id order = pine1..pine18)
        known_trees_ = {
            {1,  0.0, -10.0, 0.0, 0},
            {2,  0.0,  -6.0, 0.0, 0},
            {3,  0.0,  -2.0, 0.0, 0},
            {4,  0.0,   2.0, 0.0, 0},
            {5,  0.0,   6.0, 0.0, 0},
            {6,  0.0,  10.0, 0.0, 0},
            {7, -4.0, -10.0, 0.0, 0},
            {8, -4.0,  -6.0, 0.0, 0},
            {9, -4.0,  -2.0, 0.0, 0},
            {10,-4.0,   2.0, 0.0, 0},
            {11,-4.0,   6.0, 0.0, 0},
            {12,-4.0,  10.0, 0.0, 0},
            {13, 4.0, -10.0, 0.0, 0},
            {14, 4.0,  -6.0, 0.0, 0},
            {15, 4.0,  -2.0, 0.0, 0},
            {16, 4.0,   2.0, 0.0, 0},
            {17, 4.0,   6.0, 0.0, 0},
            {18, 4.0,  10.0, 0.0, 0}
        };

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarTreeDetectorNode::scan_callback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10, std::bind(&LidarTreeDetectorNode::odom_callback, this, std::placeholders::_1));
        pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "known_tree_widths", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "detected_trees_markers", 10);
    
        RCLCPP_INFO(get_logger(), "lidar_tree_detector_minimal started");
    }

private:
    // Tunables
    static constexpr double MAX_RANGE = 15.0;
    static constexpr double CLUSTER_EPS = 0.6;   // cluster distance threshold (meters)
    static constexpr size_t CLUSTER_MIN = 3;     // min points per cluster
    static constexpr double MATCH_RADIUS = 1.0;  // match cluster to known tree if within this radius

    // known trees
    std::vector<KnownTree> known_trees_;

    // robot pose 
    mutable std::mutex pose_mtx_;
    double robot_x_ = std::numeric_limits<double>::quiet_NaN();
    double robot_y_ = std::numeric_limits<double>::quiet_NaN();
    double robot_yaw_ = std::numeric_limits<double>::quiet_NaN();

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;
        double yaw = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
        std::lock_guard<std::mutex> lk(pose_mtx_);
        robot_x_ = p.x;
        robot_y_ = p.y;
        robot_yaw_ = yaw;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // get current pose copy
        double rx, ry, ryaw;
        {
            std::lock_guard<std::mutex> lk(pose_mtx_);
            rx = robot_x_; ry = robot_y_; ryaw = robot_yaw_;
        }
        bool have_pose = !(std::isnan(rx) || std::isnan(ry) || std::isnan(ryaw));

        // transform each valid lidar point into world frame using robot pose
        std::vector<std::pair<double,double>> pts;
        pts.reserve(msg->ranges.size());
        double angle = msg->angle_min;
        for (const auto &r : msg->ranges) {
            if (!(std::isfinite(r) && r > 0.0 && r <= MAX_RANGE)) { angle += msg->angle_increment; continue; }
            double lx = r * std::cos(angle);
            double ly = r * std::sin(angle);
            if (have_pose) {
                double c = std::cos(ryaw), s = std::sin(ryaw);
                double wx = rx + (c * lx - s * ly);
                double wy = ry + (s * lx + c * ly);
                pts.emplace_back(wx, wy);
            } else {
                // no odom yet: use scan-local coordinates 
                pts.emplace_back(lx, ly);
            }
            angle += msg->angle_increment;
        }

        // cluster points (simple region growing / DBSCAN-like)
        auto clusters = cluster_points(pts);

        // For each cluster compute centroid and width
        for (const auto &cluster : clusters) {
            if (cluster.size() < CLUSTER_MIN) continue;
            auto cen = cluster_centroid(cluster);
            double width = cluster_width(cluster);

            // match cluster to known trees
            for (auto &kt : known_trees_) {
                double d = std::hypot(cen.first - kt.x, cen.second - kt.y);
                if (d <= MATCH_RADIUS) {
                    kt.width_sum += width;
                    kt.width_count += 1;
                }
            }
        }

        // publish current averages for any known trees that have data
        publish_known_tree_widths();
    }

    // clustering
    std::vector<std::vector<std::pair<double,double>>> cluster_points(const std::vector<std::pair<double,double>>& pts) const
    {
        std::vector<std::vector<std::pair<double,double>>> clusters;
        size_t n = pts.size();
        if (n == 0) return clusters;
        std::vector<char> used(n, 0);
        for (size_t i = 0; i < n; ++i) {
            if (used[i]) continue;
            std::vector<size_t> stack{ i };
            used[i] = 1;
            std::vector<std::pair<double,double>> cluster;
            while (!stack.empty()) {
                size_t idx = stack.back(); stack.pop_back();
                cluster.push_back(pts[idx]);
                for (size_t j = 0; j < n; ++j) {
                    if (used[j]) continue;
                    if (std::hypot(pts[j].first - pts[idx].first, pts[j].second - pts[idx].second) <= CLUSTER_EPS) {
                        used[j] = 1;
                        stack.push_back(j);
                    }
                }
            }
            if (cluster.size() >= CLUSTER_MIN) clusters.push_back(std::move(cluster));
        }
        return clusters;
    }

    // simple centre (mean)
    std::pair<double,double> cluster_centroid(const std::vector<std::pair<double,double>>& pts) const
    {
        double sx = 0.0, sy = 0.0;
        for (const auto &p : pts) { sx += p.first; sy += p.second; }
        double n = static_cast<double>(pts.size());
        return { sx / n, sy / n };
    }

    // compute width as max distance between any two points in cluster
    double cluster_width(const std::vector<std::pair<double,double>>& pts) const
    {
        double maxd = 0.0;
        for (size_t i = 0; i < pts.size(); ++i)
            for (size_t j = i + 1; j < pts.size(); ++j)
                maxd = std::max(maxd, std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second));
        return maxd;
    }

        void publish_known_tree_widths()
    {
        std_msgs::msg::Int32MultiArray msg;
        // collect rows for trees that have measurements
        std::vector<int> flat;
        int rows = 0;
        for (const auto &kt : known_trees_) {
            if (kt.width_count == 0) continue;
            double avg = kt.width_sum / static_cast<double>(kt.width_count);
            int width_i = static_cast<int>(std::lround(avg*100));
            int x_i = static_cast<int>(std::lround(kt.x));
            int y_i = static_cast<int>(std::lround(kt.y));
            flat.push_back(width_i);
            flat.push_back(x_i);
            flat.push_back(y_i);
            rows++;
        }

        // Setup layout: dims[0]=rows, dims[1]=3
        std_msgs::msg::MultiArrayDimension dim0;
        std_msgs::msg::MultiArrayDimension dim1;
        dim0.label = "rows";
        dim0.size = static_cast<size_t>(rows);
        dim0.stride = static_cast<size_t>(rows * 3);
        dim1.label = "fields";
        dim1.size = 3;
        dim1.stride = 3;
        msg.layout.dim.clear();
        if (rows > 0) {
            msg.layout.dim.push_back(dim0);
            msg.layout.dim.push_back(dim1);
        }
        msg.data = std::move(flat);
        pub_->publish(msg);

        // write CSV atomically for logging
        save_csv();

        // build and publish visualization markers for RViz
        visualization_msgs::msg::MarkerArray ma;
        rclcpp::Time now = this->now();
        for (const auto &kt : known_trees_) {
            if (kt.width_count == 0) continue;
            double avg = kt.width_sum / static_cast<double>(kt.width_count);
            int width_i = static_cast<int>(std::lround(avg*100));
            int x_i = static_cast<int>(std::lround(kt.x));
            int y_i = static_cast<int>(std::lround(kt.y));

            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = now;
            m.ns = "detected_trees";
            m.id = kt.id; // tree id
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;

            m.pose.position.x = static_cast<double>(x_i);
            m.pose.position.y = static_cast<double>(y_i);
            m.pose.position.z = 2.0; 
            m.pose.orientation.w = 1.0;

            // text: "<width>cm"
            std::ostringstream oss;
            oss << width_i << "cm";
            m.text = oss.str();

            // scale
            m.scale.z = 0.35f;

            // color (green)
            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;

            // lifetime
            m.lifetime = builtin_interfaces::msg::Duration();

            ma.markers.push_back(m);
        }

        // publish marker array
        if (!ma.markers.empty()) {
            marker_pub_->publish(ma);
        }
    }

    void save_csv()
    {
        std::ofstream ofs("detected_trees.csv", std::ofstream::out | std::ofstream::trunc);
        if (!ofs.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open detected_trees.csv for writing");
            return;
        }
        ofs << "id,width,center_x,center_y,samples\n";
        for (const auto &kt : known_trees_) {
            int width_i = 0;
            if (kt.width_count > 0) {
                double avg = kt.width_sum / static_cast<double>(kt.width_count);
                width_i = static_cast<int>(std::lround(avg*100));
            }
            int x_i = static_cast<int>(std::lround(kt.x));
            int y_i = static_cast<int>(std::lround(kt.y));
            ofs << kt.id << ',' << width_i << ',' << x_i << ',' << y_i << ',' << kt.width_count << '\n';
        }
        ofs.close();
    }
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarTreeDetectorNode>());
    rclcpp::shutdown();
    return 0;
}