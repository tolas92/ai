import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion,Point,Pose
from tf2_ros import TransformBroadcaster
import math
from visualization_msgs.msg import MarkerArray

class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('Person_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        #self.pose_subscriber=self.create_subscription(Pose,"/person_pose",self.callback,1)
        self.timer = self.create_timer(0.1, self.tf_callback)
        self.subscription = self.create_subscription(
            MarkerArray,
            '/spatial_bb',
            self.callback,
            10)
        self.camera_frame = 'camera_link'
        self.camera_optical_frame = 'object_link'
        self.position_x=None
        self.position_y=None

    def callback(self,msg):
        for marker in msg.markers:
            if marker.text == 'person':        
                # Set the translation and rotation from the constant pose
                self.position_x = marker.pose.position.z
                self.position_y = -(marker.pose.position.x)
            
    def tf_callback(self):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = self.camera_frame
        transform_stamped.child_frame_id = self.camera_optical_frame
        if self.position_x and self.position_y is not None:
        # Set the translation and rotation from the constant pose
            transform_stamped.transform.translation.x = self.position_x
            transform_stamped.transform.translation.y = -(self.position_y)
            transform_stamped.transform.translation.z = 0.0

            # Set the rotation (quaternion)
            transform_stamped.transform.rotation.x = 0.0
            transform_stamped.transform.rotation.y = 0.0
            transform_stamped.transform.rotation.z = 0.0
            transform_stamped.transform.rotation.w = 1.0

            # Publish the transformation
            self.tf_broadcaster.sendTransform(transform_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
