import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Follow the Gap on the car
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribe to LIDAR
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        
        # Publish to drive
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array:
            1. Setting each value to the mean over some window
            2. Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.array(ranges)
        proc_ranges = np.convolve(proc_ranges, np.ones(5)/5, mode='same')  # Mean filter
        proc_ranges[proc_ranges > 3.0] = 3.0  # Reject high values
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        free_space = np.where(free_space_ranges > 0)[0]
        if len(free_space) == 0:
            return None, None

        max_gap = (0, 0)
        current_gap = (free_space[0], free_space[0])

        for i in range(1, len(free_space)):
            if free_space[i] == free_space[i-1] + 1:
                current_gap = (current_gap[0], free_space[i])
            else:
                if current_gap[1] - current_gap[0] > max_gap[1] - max_gap[0]:
                    max_gap = current_gap
                current_gap = (free_space[i], free_space[i])

        if current_gap[1] - current_gap[0] > max_gap[1] - max_gap[0]:
            max_gap = current_gap

        return max_gap

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        best_point = np.argmax(ranges[start_i:end_i]) + start_i
        return best_point

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # Find closest point to LiDAR
        closest_point = np.argmin(proc_ranges)
        
        # Eliminate all points inside 'bubble' (set them to zero)
        bubble_radius = int(np.radians(30) / data.angle_increment)  # Assuming 30 degrees in radians
        start_bubble = max(0, closest_point - bubble_radius)
        end_bubble = min(len(proc_ranges) - 1, closest_point + bubble_radius)
        proc_ranges[start_bubble:end_bubble] = 0

        # Find max length gap
        start_i, end_i = self.find_max_gap(proc_ranges)
        if start_i is None or end_i is None:
            self.get_logger().info("No gap found!")
            return

        # Find the best point in the gap
        best_point = self.find_best_point(start_i, end_i, proc_ranges)

        # Publish Drive message
        angle = (best_point - len(ranges) / 2) * data.angle_increment
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = 1.0  # Set a constant speed

        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("ReactiveFollowGap Node Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
