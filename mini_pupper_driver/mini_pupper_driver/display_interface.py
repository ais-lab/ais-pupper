import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
from MangDang.LCD.ST7789 import ST7789

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_interface')
        self.get_logger().info("Initializing display interface")
        self.sub = self.create_subscription(CompressedImage, 'mini_pupper_lcd/image/compressed', self.callback, 10)

        # Replace by this subscriber if image not compressed needed
        #self.sub = self.create_subscription(Image, 'mini_pupper_lcd/image/raw', self.callback, 10)

        self.bridge = CvBridge()
        self.get_logger().info("Creating LCD hardware interface")
        self.ST7789 = ST7789()

    def callback(self, image):
        np_arr = np.frombuffer(image.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_img is None:
            self.get_logger().error("Failed to decode image.")
            return
        image_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        self.ST7789.display(image_rgb)
        self.get_logger().info("Media received and displayed")

def main(args=None): 
    rclpy.init(args=args)
    display_node = DisplayNode()
    rclpy.spin(display_node)
    display_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()