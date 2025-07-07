import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import math
 
class StaticTransformPublisher(Node):
 
    def __init__(self):
        super().__init__('static_transform_publisher')
        self._tf_publisher = StaticTransformBroadcaster(self)
        
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'map'  # Global frame
        transform_stamped.child_frame_id = 'base_link'  # Frame of your PointCloud2
        # transform_stamped.child_frame_id = 'camera_depth_optical_frame'  # Frame of your PointCloud2
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0
        # transform_stamped.transform.rotation.x = -1/math.sqrt(2)
        # transform_stamped.transform.rotation.y = 0.0
        # transform_stamped.transform.rotation.z = 0.0
        # transform_stamped.transform.rotation.w = 1/math.sqrt(2)
 
        self._tf_publisher.sendTransform(transform_stamped)
        self.get_logger().info('Publishing static transform from map to base_link')
 
def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
