#!/usr/bin/env python3

from collections import namedtuple
from copy import deepcopy
from math import atan2, pi, sqrt, asin, radians
from statistics import mean
from threading import Lock, Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point
from std_srvs.srv import SetBool

from robot_interfaces.srv import SetPIDPlannerPrecision

from pid_controller_lib import PID_Controller

### helpers ##################################################################

StampedPose = namedtuple('StampedPose', 'pose time')

EulerAngle = namedtuple('EulerAngle', 'roll pitch yaw')

def clamp(value: float, lower: float, upper: float) -> float:
	return float(min(upper, max(value, lower)))

def euler_from_quaternion(x: float, y: float, z: float, w: float) -> EulerAngle:
	"""Return euler angles (roll, pitch, yaw) of given quaternion angle."""

	t0 = 2 * (w * x + y * z)
	t1 = 1 - 2 * (x * x + y * y)
	roll = atan2(t0, t1)

	t2 = 2 * (w * y - z * x)
	t2 = clamp(t2, -1, 1)
	pitch = asin(t2)

	t3 = 2 * (w * z + x * y)
	t4 = 1 - 2 * (y * y + z * z)
	yaw = atan2(t3, t4)

	return EulerAngle(roll, pitch, yaw)

def npi_to_pi_angle(angle):
	if angle > pi:
		return angle - 2*pi
	elif angle < -pi:
		return angle + 2*pi
	return angle

def pose_to_waypoint_error(pose: Pose, waypoint: Point):
	# find the error in cartesian coordinates
	x_error = waypoint.x - pose.position.x
	y_error = waypoint.y - pose.position.y
	dist_error = sqrt(x_error**2 + y_error**2)

	# find heading
	ori = pose.orientation
	euler = euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
	heading = euler.yaw
	
	# calculate the heading that would point at the waypoint
	heading_to_waypoint = atan2(y_error, x_error)

	# find the heading error (choose smaller direction of rotation)
	heading_error = heading_to_waypoint - heading
	heading_error = npi_to_pi_angle(heading_error)

	return dist_error, heading_error

def dist2d(pt1: Point, pt2: Point):
	x_error = pt1.x - pt2.x
	y_error = pt1.y - pt2.y
	return sqrt(x_error**2 + y_error**2)

class PID_Planner(Node):
	def __init__(self) -> None:

		super().__init__('pid_planner')
		
        ### declare params ###################################################

		# NOTE these defaults are based on the values used for Wanderer for URC 2023
		self.declare_parameter("dist_prop_gain", 0.1)
		self.declare_parameter("dist_kp", 0.04)
		self.declare_parameter("dist_kd", 0.0)
		self.declare_parameter("dist_ki", 0.1)

		self.declare_parameter("theta_prop_gain", 0.1)
		self.declare_parameter("head_kp", 0.1)
		self.declare_parameter("head_kd", 0.0)
		self.declare_parameter("head_ki", 0.0)

		self.declare_parameter("min_linear_speed", 0.0)
		self.declare_parameter("max_linear_speed", 1.0)
		self.declare_parameter("min_angular_speed", -1.0)
		self.declare_parameter("max_angular_speed", 1.0)

		self.declare_parameter("angle_threshold", 10.0)
		self.declare_parameter("default_precision", 1.0)
		self.declare_parameter("update_frequency", 10.0)
		self.declare_parameter("not_moving_threshold", 1.0) 
		self.declare_parameter("not_moving_delay", 5.0)
		self.declare_parameter("stuck_delay", 3.0)
		
		self.declare_parameter("current_waypoint_topic", "/current_waypoint")
		self.declare_parameter("local_position_topic", "/rover_pose")
		self.declare_parameter("cmd_vel_topic", "/cmd_vel")
		self.declare_parameter("current_status_topic", "/planner_status")
		self.declare_parameter("enabled_service", "/enable_planner")
		self.declare_parameter("precision_service", "/set_planner_precision")

		### prepare PID controllers ##########################################

		dist_prop_gain = self.get_parameter("dist_prop_gain").get_parameter_value().double_value
		dist_kp = self.get_parameter("dist_kp").get_parameter_value().double_value
		dist_kd = self.get_parameter("dist_kd").get_parameter_value().double_value
		dist_ki = self.get_parameter("dist_ki").get_parameter_value().double_value
		self.dist_pid_controller = PID_Controller(dist_prop_gain, dist_kp, dist_kd, dist_ki)

		theta_prop_gain = self.get_parameter("theta_prop_gain").get_parameter_value().double_value
		head_kp = self.get_parameter("head_kp").get_parameter_value().double_value
		head_kd = self.get_parameter("head_kd").get_parameter_value().double_value
		self.get_logger().info(f"KP: {head_kp}")
		self.get_logger().info(f"KD: {head_kd}")
		head_ki = self.get_parameter("head_ki").get_parameter_value().double_value
		self.head_pid_controller = PID_Controller(theta_prop_gain, head_kp, head_kd, head_ki)

		self.pid_lock = Lock()
		
		### local variables ##################################################

		self.min_linear_speed = self.get_parameter("min_linear_speed").get_parameter_value().double_value
		self.max_linear_speed = self.get_parameter("max_linear_speed").get_parameter_value().double_value
		self.min_angular_speed = self.get_parameter("min_angular_speed").get_parameter_value().double_value
		self.max_angular_speed = self.get_parameter("max_angular_speed").get_parameter_value().double_value

		self.angle_threshold = radians(self.get_parameter("angle_threshold").get_parameter_value().double_value)
		self.get_logger().info(f"ANG_THRESH: {self.angle_threshold}")
		default_precision = self.get_parameter("default_precision").get_parameter_value().double_value
		self.freq = self.get_parameter("update_frequency").get_parameter_value().double_value
		self.not_moving_threshold = self.get_parameter("not_moving_threshold").get_parameter_value().double_value
		self.not_moving_delay = Duration(seconds=self.get_parameter("not_moving_delay").get_parameter_value().double_value)
		self.stuck_delay = Duration(seconds=self.get_parameter("stuck_delay").get_parameter_value().double_value)

		self.enabled_lock = Lock()
		self.enabled = False  # whether or not to publish cmd_vel

		self.precision_lock = Lock()
		self.precision = default_precision

		self.start_move_time_lock = Lock()
		self.start_move_time = None

		self.waypoint_lock = Lock()
		self.current_waypoint = None

		self.pose_lock = Lock()
		self.current_pose = None

		self.recent_positions_window_lock = Lock()
		self.recent_positions_window = []

		self.not_moving_lock = Lock()
		self.not_moving = True

		self.stuck_lock = Lock()
		self.stuck = False

		### connect to ROS ###################################################

		current_waypoint_topic = self.get_parameter("current_waypoint_topic").get_parameter_value().string_value
		local_position_topic = self.get_parameter("local_position_topic").get_parameter_value().string_value
		cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
		current_status_topic = self.get_parameter("current_status_topic").get_parameter_value().string_value
		enabled_service_name = self.get_parameter("enabled_service").get_parameter_value().string_value
		precision_service_name = self.get_parameter("precision_service").get_parameter_value().string_value

		qos = QoSProfile(
			depth=10,
			reliability=QoSReliabilityPolicy.BEST_EFFORT
		)

		self.current_waypoint_sub = self.create_subscription(Point, current_waypoint_topic, self.current_waypoint_sub_callback, 10)
		self.pose_sub = self.create_subscription(Pose, local_position_topic, self.pose_sub_callback, 10)
		self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10) # TODO check if we actually want 1
		self.current_status_pub = self.create_publisher(String, current_status_topic, 10) # check if we actually want 1
		self.enabled_service = self.create_service(SetBool, enabled_service_name, self.enabled_callback)
		self.precision_service = self.create_service(SetPIDPlannerPrecision, precision_service_name, self.precision_callback)

		### end init #########################################################

	### local functions ######################################################

	def publish_status(self):
		# +------------+----------+-------------+
		# |            | waypoint | no waypoint |
		# +------------+-----------+------------+
		# | ability    | active   | idle        |
		# | no ability | blocked  | inactive    |
		# +------------+----------+-------------+

		with self.enabled_lock:
			enabled = self.enabled
		with self.waypoint_lock:
			has_waypoint = self.current_waypoint is not None
		with self.pose_lock:
			has_location = self.current_pose is not None
		with self.stuck_lock:
			stuck = self.stuck

		has_ability = enabled and has_location

		status = String()

		if has_ability and has_waypoint:
			if stuck:
				status.data = "stuck"
			else:
				status.data = "active"
		elif has_ability and not has_waypoint:
			status.data = "idle"
		elif not has_ability and has_waypoint:
			status.data = "blocked"
		elif not has_ability and not has_waypoint:
			status.data = "inactive"

		self.current_status_pub.publish(status)

	### callbacks ############################################################

	def current_waypoint_sub_callback(self, waypoint: Point):
		with self.pose_lock:
			current_position = deepcopy(self.current_pose.position) # TODO NoneType problem here
		with self.waypoint_lock:
			current_waypoint = deepcopy(self.current_waypoint)
		with self.precision_lock:
			precision = deepcopy(self.precision)

		# check if already at waypoint
		at_waypoint = dist2d(current_position, waypoint) < precision

		# check if waypoint is significantly different than current target
		new_target = not current_waypoint or dist2d(waypoint, current_waypoint) > precision

		if not at_waypoint:
			# update target waypoint
			with self.waypoint_lock:
				self.current_waypoint = waypoint

		if not at_waypoint and new_target:
			# reset pid errors
			with self.pid_lock:
				self.dist_pid_controller.reset()
				self.head_pid_controller.reset()
			with self.start_move_time_lock:
				self.start_move_time = self.get_clock().now()

		self.publish_status()

	def enabled_callback(self, request, response):
		enabled = request.data
		with self.enabled_lock:
			being_disabled = self.enabled and not enabled
			being_enabled = not self.enabled and enabled
			self.enabled = enabled
		self.publish_status()

		if being_disabled:
			self.cmd_vel_pub.publish(Twist())

		if being_enabled:
			with self.start_move_time_lock:
				self.start_move_time = self.get_clock().now()

		response.success = True
		response.message = "Updated pid path planner's enable status"
		return response

	def precision_callback(self, request, response):
		precision = request.precision
		with self.precision_lock:
			self.precision = precision

		response.success = True
		response.message = f"Updated pid path planner's precision to {precision}"
		return response

	def pose_sub_callback(self, pose_msg: Pose):
		with self.pose_lock:
			self.current_pose = pose_msg

		with self.recent_positions_window_lock:
			# add position to recent position window
			self.recent_positions_window.append(StampedPose(pose_msg, self.get_clock().now()))

			# remove positions too old to be in recent position window
			updated_window = []
			for stamped_pose in self.recent_positions_window:
				if stamped_pose.time + self.not_moving_delay > self.get_clock().now():
					updated_window.append(stamped_pose)

			self.recent_positions_window = updated_window

		# check if we are not moving by comparing current position to average position of position window
		with self.not_moving_lock:
			positions_in_window = [stamped_pose.pose.position for stamped_pose in updated_window]
			x_mean = float(mean([position.x for position in positions_in_window] or [0.0]))
			y_mean = float(mean([position.y for position in positions_in_window] or [0.0]))
			z_mean = float(mean([position.z for position in positions_in_window] or [0.0]))
			average_position = Point()

			average_position.x = x_mean
			average_position.y = y_mean
			average_position.z = z_mean

			self.not_moving = dist2d(average_position, pose_msg.position) * 2 < self.not_moving_threshold


### main #####################################################################

def main(args=None):
	rclpy.init(args=args)
	planner_node = PID_Planner()

	# spin in thread to keep ROS callbacks happy
	spin_thread = Thread(target=rclpy.spin, args=(planner_node, ), daemon=True)
	spin_thread.start()

	rate = planner_node.create_rate(2)
	# rate = planner_node.create_rate(planner_node.freq, planner_node.get_clock())

	try:
		while rclpy.ok():
			# ensure pid errors are not reset until after current error calculations are applied
			with planner_node.pid_lock:

				with planner_node.enabled_lock:
					enabled = planner_node.enabled
				with planner_node.precision_lock:
					precision = planner_node.precision
				with planner_node.waypoint_lock:
					current_waypoint = deepcopy(planner_node.current_waypoint)
				with planner_node.pose_lock:
					current_pose = deepcopy(planner_node.current_pose)
				with planner_node.not_moving_lock:
					not_moving = planner_node.not_moving
				with planner_node.start_move_time_lock:
					start_move_time = planner_node.start_move_time
				if enabled and current_waypoint and current_pose:

					# calculate error between current pose and waypoint
					dist_error, head_error = pose_to_waypoint_error(current_pose, current_waypoint)

				#	planner_node.get_logger().info(f'Distance Error: {dist_error}')
				#	planner_node.get_logger().info(f'Heading Error: {head_error}')

					if dist_error < precision:
						planner_node.get_logger().info('WAYPOINT REACHED')
						with planner_node.waypoint_lock:
							planner_node.current_waypoint = None
						planner_node.cmd_vel_pub.publish(Twist())
						planner_node.publish_status()
						continue

					linear = planner_node.dist_pid_controller.update(dist_error)
					angular = planner_node.head_pid_controller.update(head_error)

				#	planner_node.get_logger().info(f'Linear: {linear}')
				#	planner_node.get_logger().info(f'Angular: {angular}')


				#	planner_node.get_logger().info(f"Heading Threshold {planner_node.angle_threshold}")
					# don't move forward if we still need to turn a lot
					if abs(head_error) > planner_node.angle_threshold:
						planner_node.get_logger().info(f"Turning: {head_error}")
						linear = 0.0
					else:
						planner_node.get_logger().info(f"Heading Good: {head_error}")

					# check if we are stuck
					# should_be_moving_by_now = planner_node.get_clock().now() > start_move_time + planner_node.stuck_delay
					# stuck = not_moving and should_be_moving_by_now
					# with planner_node.stuck_lock:
					# 	planner_node.stuck = stuck

					# move at full speed forward if stuck
					# if stuck:
						# planner_node.get_logger().info('STUCK!!!')
						# linear, angular = planner_node.max_linear_speed, 0.0

					# scale cmd_vel and publish
					cmd_vel = Twist()
					cmd_vel.linear.x = clamp(linear, planner_node.min_linear_speed, planner_node.max_linear_speed)
					cmd_vel.angular.z = clamp(angular, planner_node.min_angular_speed, planner_node.max_angular_speed)
					planner_node.cmd_vel_pub.publish(cmd_vel)
					planner_node.publish_status()

			# run the loop at the specified rate
			rate.sleep()

	except KeyboardInterrupt:
		pass

	planner_node.destroy_node()
	rclpy.shutdown()
	spin_thread.join()
	
if __name__ == '__main__':
	main()
