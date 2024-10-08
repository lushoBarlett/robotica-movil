#!/usr/bin/env python3

"""
ROS node for 2D cylinder detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from math import cos, sin, pi, sqrt

class CylinderDetector(Node):
    """ Node class """
    def __init__(self):
        super().__init__('cylinder_detector_node')
        self.create_subscription(LaserScan, '/scan', self.laser_cb, 10)

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

    def _get_itervals(self, ranges, max_range):
        """ Function to obtain all the intervals in the laser ranges
        that belong to a unique cylinder detection """
        intervals = []
        ranges_len = len(ranges)

        start = -1
        finish = 0
        for i, r in enumerate(ranges):
            if r < max_range and start == -1:
                start = i

            if r > max_range and start != -1:
                finish = i-1
                intervals += [(start, finish)]
                start = -1

        # -- Corner cases handling --

        # Started detecting a cylinder that finishes right in the
        # last position
        if start != -1 and ranges[0] > max_range:
            finish = ranges_len - 1
            intervals += [(start, finish)]

        # The last and first interval belong to the same cylinder
        if start != -1 and ranges[0] < max_range and len(intervals) > 0:
            intervals[0] = (start, intervals[0][1])

        # There is only one detection that takes all laser scans
        # (should never happen)
        if start != -1 and len(intervals) == 0:
            intervals = [(0, ranges_len - 1)]

        return intervals

    def place_marker(self, x, y, radius, id):
        """ Places a cylinder marker of given radius in
        given position """
        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.ns = "laser_points"
        marker.id = id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2*radius
        marker.scale.y = 2*radius
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # Give some lifetime to the marker so there's a chance
        # to see them all in the same picture
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(0.2 * 1e9)
        self.marker_pub.publish(marker)

    def laser_cb(self, msg):
        """ Laser callback. Finds the laser ranges intervals that belong to
        different cylinders. Estimates radius and center of cylinders.
        Finally places a cylinder marker for each of them."""
        ranges = msg.ranges

        ranges_len = len(ranges)

        intervals = self._get_itervals(ranges, msg.range_max)

        for i, (st, end) in enumerate(intervals):
            st_angle = (st + 1) * msg.angle_increment
            st_length = ranges[st]

            end_angle = (end + 1) * msg.angle_increment
            end_length = ranges[end]

            # -- Radius calculation --
            l = (st_length + end_length) / 2
            if st < end:
                angle = abs(end_angle - st_angle)
            else:
                angle = 2 * pi - st_angle + end_angle

            diameter = l * sqrt(2 * (1 - cos(angle)))
            radius = diameter/2

            # -- Center calculation --
            middle_point_idx = 0
            if st < end:
                middle_point_idx = (end+st) // 2
            else:
                middle_point_idx = ((ranges_len-st + end) // 2 + st) % ranges_len

            center_angle = (middle_point_idx) * msg.angle_increment
            center_length = ranges[middle_point_idx] + radius

            center_x = center_length * cos(center_angle)
            center_y = center_length * sin(center_angle)

            # -- Marker placing --
            self.place_marker(center_x, center_y, radius, i)


def main(args=None):
    rclpy.init(args=args)

    node = CylinderDetector()

    rclpy.spin(node)

    rclpy.shutdown()

    node.get_clock().now()


if __name__ == '__main__':
    main()
