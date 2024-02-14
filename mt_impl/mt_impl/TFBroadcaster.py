import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        self._broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(2.0, self.publish_transforms)

    def publish_transforms(self):
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'map'
        static_transformStamped.child_frame_id = 'odom'

        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0,0,0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._broadcaster.sendTransform(static_transformStamped)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()