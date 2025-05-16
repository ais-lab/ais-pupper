import time
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CompressedImage
import subprocess

class MiniPupperDetectFace(Node):
    def __init__(self):
        super().__init__('mini_pupper_detect_face')
        self.get_logger().info('Mini Pupper Detect Face Node Initialized')

        self.sub = self.create_subscription(
            Detection2DArray,
            'detected_faces',
            self.detect_face_callback,
            10
        )
        self.pub = self.create_publisher(
            CompressedImage,
            'mini_pupper_lcd/image/compressed',
            10
        )
        self.get_logger().info('Listening to detected faces...')
        self.face_detected = False
        self.no_face_reaction_img_path = '/home/ubuntu/ros2_ws/src/mini_pupper_ros/robot_state_folder/trot.png'
        self.face_reaction_img_path = '/home/ubuntu/ros2_ws/src/mini_pupper_ros/robot_state_folder/hop.png'
    
    def send_media(self, media_path):
        try:
            subprocess.run([
                "ros2", "run", "media_publisher", "media_sender_node", media_path
            ], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Failed to send media: {e}")

    def detect_face_callback(self, msg):
        """
        Callback function to handle detected faces.
        """
        if len(msg.detections) > 0:
            self.get_logger().info('Face detected!')
            if self.face_detected == False:
                self.send_media(self.face_reaction_img_path)
                self.send_media(self.face_reaction_img_path)
                self.send_media(self.face_reaction_img_path)
            self.face_detected = True
        else:
            self.get_logger().info('No face detected.')
            if self.face_detected == True:
                self.send_media(self.no_face_reaction_img_path)
                self.send_media(self.no_face_reaction_img_path)
                self.send_media(self.no_face_reaction_img_path)
            self.face_detected = False   
        

def main(args=None):
    rclpy.init(args=args)
    mini_pupper_detect_face = MiniPupperDetectFace()
    rclpy.spin(mini_pupper_detect_face)
    mini_pupper_detect_face.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()