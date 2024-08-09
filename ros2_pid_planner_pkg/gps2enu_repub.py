import pymap3d as pm

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Point

class Gps2EnuRepub(Node):

    def __init__(self):
        super().__init__('gps2enu_repub_node')
        self.global_origin_latitude = None
        self.global_origin_longitude = None
        self.global_origin_altitude = None

        self.gps_waypoint_sub = self.create_subscription(GeoPoint, '/gps_waypoint', self.convert_callback, 10)
        self.global_origin_sub = self.create_subscription(GeoPoint, '/global_origin', self.get_global_origin_callback, 10)

        self.enu_waypoint_pub = self.create_publisher(Point, '/current_waypoint', 10)

    def convert_callback(self, msg: GeoPoint):
        if self.global_origin_latitude and self.global_origin_longitude and self.global_origin_altitude:
            waypoint = Point()
            ### translate geodetic coords to enu ###
            lat = msg.latitude
            lon = msg.longitude
            h = msg.altitude
            lat0 = self.global_origin_latitude
            lon0 = self.global_origin_longitude
            h0 = self.global_origin_altitude
            
            e, n, u = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
            waypoint = Point()
            waypoint.x = e
            waypoint.y = n
            waypoint.z = u

            self.enu_waypoint_pub.publish(waypoint)

    def get_global_origin_callback(self, msg: GeoPoint):
        if msg:
            self.global_origin_latitude = msg.latitude
            self.global_origin_longitude = msg.longitude
            self.global_origin_altitude = msg.altitude

def main(args=None):
    rclpy.init(args=args)

    convert_node = Gps2EnuRepub()

    rclpy.spin(convert_node)

    # Destroy the node explicitly
    convert_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
