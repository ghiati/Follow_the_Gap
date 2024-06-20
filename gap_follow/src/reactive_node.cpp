#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std;

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap() : Node("reactive_node") {
        // Create ROS subscribers and publishers
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic_, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    }

private:
    string lidarscan_topic_ = "/scan";
    string drive_topic_ = "/drive";
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    vector<float> preprocess_lidar(const vector<float> &ranges) {
        // Preprocess the LiDAR scan array
        vector<float> proc_ranges = ranges;
        size_t len = proc_ranges.size();

        // Mean filter
        vector<float> filtered_ranges(len);
        for (size_t i = 2; i < len - 2; ++i) {
            filtered_ranges[i] = (proc_ranges[i - 2] + proc_ranges[i - 1] + proc_ranges[i] + proc_ranges[i + 1] + proc_ranges[i + 2]) / 5;
        }

        // Reject high values
        transform(filtered_ranges.begin(), filtered_ranges.end(), filtered_ranges.begin(),
                       [](float range) { return (range > 3.0) ? 3.0 : range; });

        return filtered_ranges;
    }

    pair<int, int> find_max_gap(const vector<float> &ranges) {
        // Return the start index & end index of the max gap in free_space_ranges
        int max_start = 0, max_end = 0;
        int current_start = 0, current_end = 0;
        bool in_gap = false;

        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > 0) {
                if (!in_gap) {
                    in_gap = true;
                    current_start = i;
                }
                current_end = i;
            } else {
                if (in_gap) {
                    in_gap = false;
                    if ((current_end - current_start) > (max_end - max_start)) {
                        max_start = current_start;
                        max_end = current_end;
                    }
                }
            }
        }

        if (in_gap && (current_end - current_start) > (max_end - max_start)) {
            max_start = current_start;
            max_end = current_end;
        }

        return {max_start, max_end};
    }

    int find_best_point(int start_i, int end_i, const vector<float> &ranges) {
        // Return index of best point in ranges
        return distance(ranges.begin(), max_element(ranges.begin() + start_i, ranges.begin() + end_i));
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        vector<float> ranges = scan_msg->ranges;
        vector<float> proc_ranges = preprocess_lidar(ranges);

        // Find closest point to LiDAR
        auto min_it = min_element(proc_ranges.begin(), proc_ranges.end());
        int closest_point = distance(proc_ranges.begin(), min_it);

        // Eliminate all points inside 'bubble' (set them to zero)
        int bubble_radius = static_cast<int>(round(30.0 * M_PI / 180.0 / scan_msg->angle_increment));
        int start_bubble = max(0, closest_point - bubble_radius);
        int end_bubble = min(static_cast<int>(proc_ranges.size()) - 1, closest_point + bubble_radius);
        fill(proc_ranges.begin() + start_bubble, proc_ranges.begin() + end_bubble, 0.0);

        // Find max length gap
        auto [start_i, end_i] = find_max_gap(proc_ranges);

        // Find the best point in the gap
        int best_point = find_best_point(start_i, end_i, proc_ranges);

        // Calculate steering angle
        float angle = (best_point - proc_ranges.size() / 2) * scan_msg->angle_increment;

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = 1.0;  // Set a constant speed

        drive_pub_->publish(drive_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
