from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose as TurtlePose
from geometry_msgs.msg import Pose

class TurtlePoseTo3DPoseNode(Node):

    def __init__(self):
        super().__init__('turtle_pose_sub_node')
        self.turtle_pose_sub = self.create_subscription(
            TurtlePose,
            '/turtle1/pose',
            self.convert_callback,
            10)
        self.turtle_pose_sub  # prevent unused variable warning

        self.pose_3d_pub = self.create_publisher(Pose, 'pose_3d', 10)

    def convert_callback(self, msg: TurtlePose):
        # self.get_logger().info('Turtle pose: "%d"' % msg.num)
        pose_3d = Pose()

        # convert from euler to quaternion
        rot = Rotation.from_euler('xyz', [0, 0, msg.theta])
        rot_quat = rot.as_quat()

        # populate message
        pose_3d.position.x = msg.x
        pose_3d.position.y = msg.y
        pose_3d.position.z = 0.0 # we're in 2D

        pose_3d.orientation.x = rot_quat[0]
        pose_3d.orientation.y = rot_quat[1]
        pose_3d.orientation.z = rot_quat[2]
        pose_3d.orientation.w = rot_quat[3]

        self.get_logger().info(f'3D pose: {pose_3d}')

        self.pose_3d_pub.publish(pose_3d)

def main(args=None):
    rclpy.init(args=args)

    convert_node = TurtlePoseTo3DPoseNode()

    rclpy.spin(convert_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    convert_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
