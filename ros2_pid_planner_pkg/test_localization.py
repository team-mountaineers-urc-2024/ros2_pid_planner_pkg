#!/usr/bin/env python3

import pymap3d as pm

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix

### main #####################################################################
def main(args=None):
	rclpy.init(args=args)
	pixhawk = Pixhawk()
	
	rclpy.spin(pixhawk)
	
	# Destroy the node explicitly
	pixhawk.destroy_node()
	rclpy.shutdown()

class Pixhawk(Node):
	def __init__(self):

		super().__init__("pixhawk")

		### local variables ##################################################

		self.local_startup: Point = None
		self.global_startup: GeoPoint = None
		self.global_origin: GeoPoint = None
		self.origin_initialized = False

		### connect to ROS ###################################################

		qos = QoSProfile(
			depth=10,
			reliability=QoSReliabilityPolicy.BEST_EFFORT
		)

		self.declare_parameter("global_origin_frequency", 1.0)
		self.declare_parameter("mavros_local_pose_topic", "/mavros/local_position/pose")
		self.declare_parameter("mavros_global_position_topic", "/mavros/global_position/global")
		self.declare_parameter("local_pose_topic", "/rover_pose")
		self.declare_parameter("global_position_topic","/rover_gps_position")
		self.declare_parameter("global_origin_topic", "/global_origin")

		self.global_origin_frequency = self.get_parameter("global_origin_frequency").get_parameter_value().double_value
		mavros_local_pose_topic = self.get_parameter("mavros_local_pose_topic").get_parameter_value().string_value
		mavros_global_position_topic = self.get_parameter("mavros_global_position_topic").get_parameter_value().string_value
		local_pose_topic = self.get_parameter("local_pose_topic").get_parameter_value().string_value
		global_position_topic = self.get_parameter("global_position_topic").get_parameter_value().string_value
		global_origin_topic = self.get_parameter("global_origin_topic").get_parameter_value().string_value

		self.create_timer(self.global_origin_frequency, self.global_origin_pub_callback)

		self.local_pose_pub = self.create_publisher(Pose, local_pose_topic, 10)
		self.global_pose_pub = self.create_publisher(GeoPoint, global_position_topic, 10)
		self.global_origin_pub = self.create_publisher(GeoPoint, global_origin_topic, 10)

		self.mavros_local_pose_sub = self.create_subscription(PoseStamped, mavros_local_pose_topic, self.mavros_local_callback, qos)
		self.mavros_global_pose_sub = self.create_subscription(NavSatFix, mavros_global_position_topic, self.mavros_global_position_callback, qos)

		### end init #########################################################

	### callbacks ############################################################

	def mavros_local_callback(self, posestamped: PoseStamped):
		if not self.local_startup:
			self.local_startup = posestamped.pose.position
			self.get_logger().info("Local startup initialized")
		# self.get_logger().info(f"Rover Pose: (x: {posestamped.pose.position.x}, y: {posestamped.pose.position.y}, z: {posestamped.pose.position.z})")
		self.local_pose_pub.publish(posestamped.pose)
		
	def mavros_global_position_callback(self, gps_navsat: NavSatFix):
		gps_geopoint = GeoPoint()

		gps_geopoint.latitude = gps_navsat.latitude
		gps_geopoint.longitude = gps_navsat.longitude
		gps_geopoint.altitude = gps_navsat.altitude
		
		if not self.global_startup:
			self.global_startup = gps_geopoint
			self.get_logger().info("Global startup initialized")
		self.global_pose_pub.publish(gps_geopoint)
		# self.get_logger().info(f"Rover GPS: (lat: {gps_geopoint.latitude}, lon: {gps_geopoint.longitude}, alt: {gps_geopoint.altitude})")


	### loop #################################################################

	def global_origin_pub_callback(self):
		# once startup variables are set, calculate the global gps origin; only do this once
		if self.local_startup and self.global_startup and not self.origin_initialized:
			x, y, z = self.local_startup.x, self.local_startup.y, self.local_startup.z
			lat1, lon1, h1 = self.global_startup.latitude, self.global_startup.longitude, self.global_startup.altitude
			# the IMU may turn on before the GPS, in which case rover pose information returned by
			# mavros will start to change as the rover moves; once the GPS is initialized, the
			# following line calculates approx. where the rover was when the rover pose started being
			# updated; this syncs up the GPS and pose origin points
			lat0, lon0, h0 = pm.enu2geodetic(-x, -y, -z, lat1, lon1, h1)
			self.global_origin = GeoPoint(latitude=lat0, longitude=lon0, altitude=h0)

			# set flag to true so this only happens once
			self.get_logger().info("GLOBAL ORIGIN INTIALIZED")
			self.origin_initialized = True
		
		# publish global origin continuously
		if self.origin_initialized:
			self.global_origin_pub.publish(self.global_origin)

if __name__ == "__main__":
	main()
