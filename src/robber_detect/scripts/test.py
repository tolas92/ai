import rclpy
from vision_msgs.msg import Detection3DArray

class DetectionSubscriber:
    def __init__(self):
        self.node = rclpy.create_node('detection_subscriber')
        self.subscription = self.node.create_subscription(
            Detection3DArray,
            '/oak/nn/spatial_detections',
            self.detection_callback,
            10  # QoS profile depth
        )
        self.class_id_to_extract = '9'  # Modify this to the desired class_id

    def detection_callback(self, msg):
        for detection in msg.detections:
            if detection.results[0].hypothesis.class_id == self.class_id_to_extract:
                pose = detection.results[0].pose.pose.position
                x, y, z = pose.x, pose.y, pose.z
                print(f"Class ID: {detection.results[0].hypothesis.class_id}, X: {x}, Y: {y}, Z: {z}")

def main():
    rclpy.init()
    subscriber = DetectionSubscriber()
    rclpy.spin(subscriber.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
