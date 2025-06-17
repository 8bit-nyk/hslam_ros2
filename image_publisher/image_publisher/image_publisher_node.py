import rclpy
import cv2
import zipfile
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from io import BytesIO

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.declare_parameter('zip_path', '')
        self.declare_parameter('publish_topic', '/camera/image')
        self.declare_parameter('publish_rate', 10.0)

        self.zip_path = self.get_parameter('zip_path').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, self.publish_topic, 10)

        self.image_files = self.load_zip_images()
        self.index = 0

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_image)

    def load_zip_images(self):
        """Extract image names from ZIP without decompressing them to disk."""
        if not zipfile.is_zipfile(self.zip_path):
            self.get_logger().error(f"Invalid ZIP file: {self.zip_path}")
            return []

        with zipfile.ZipFile(self.zip_path, 'r') as archive:
            return sorted([f for f in archive.namelist() if f.lower().endswith(('.png', '.jpg', '.jpeg'))])

    def publish_image(self):
        """Extract and publish the next image from the ZIP archive."""
        if self.index >= len(self.image_files):
            self.get_logger().info("All images published.")
            return

        with zipfile.ZipFile(self.zip_path, 'r') as archive:
            with archive.open(self.image_files[self.index]) as file:
                img_array = np.frombuffer(file.read(), np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

        if img is None:
            self.get_logger().error(f"Failed to read {self.image_files[self.index]} from ZIP")
            self.index += 1
            return

        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {self.image_files[self.index]}")

        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
