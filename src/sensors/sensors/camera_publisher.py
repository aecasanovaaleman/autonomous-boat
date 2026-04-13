import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv2
import time

# This node captures frames from /dev/video0 and publishes JPEG-compressed images.
# Publishers: 'camera/image/compressed'
# Subscribers: 'emergency_stop'
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Open the camera at /dev/video0
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open /dev/video0.")
            self.destroy_node()
            return

        # Publisher for compressed camera images
        # Message type: CompressedImage
        self.image_pub = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)

        # Timer to capture and publish frames at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Subscription to emergency stop topic
        # If emergency stop is triggered, the node will be destroyed
        self.emergency_stop = self.create_subscription(
            Bool,
            'emergency_stop',
            self.destroy_node,
            10
        )

        self.get_logger().info("Camera publisher initialized on /dev/video0")

    # Capture a frame, encode as JPEG, and publish
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return

        # Encode frame as JPEG
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().warn("Failed to encode frame as JPEG")
            return

        # Build and publish the CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = jpeg.tobytes()
        self.image_pub.publish(msg)

    # Release the camera and destroy the node
    def destroy_node(self, msg=None):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        time.sleep(0.1)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        camera_publisher.get_logger().info("Shutting down camera publisher")
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
