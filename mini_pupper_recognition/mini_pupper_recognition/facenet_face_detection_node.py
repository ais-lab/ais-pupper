from facenet_pytorch import MTCNN
from sensor_msgs.msg import Image
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import torch
import cv2
import rclpy
from cv_bridge import CvBridge

class FacenetFaceDetectionNode(Node):
    """
    A class to handle the face detection using a pre-trained FaceNet model.
    """

    def __init__(self):
        """
        Initializes the FaceNetFaceDetectionNode with the necessary parameters.
        """
        # Load the pre-trained FaceNet model
        super().__init__('facenet_face_detection_node')
        self.declare_parameter('debug', False)
        self.sub = self.create_subscription(Image, '/image_raw', self.detect_faces, 10)
        self.box_publisher_ = self.create_publisher(Detection2DArray, 'detected_faces', 10)
        if self.get_parameter('debug').value:
            self.debug_publisher_ = self.create_publisher(Image, '/detected_faces/debug', 10)
        self.model = self.load_model()

    def load_model(self):
        """
        Loads the pre-trained FaceNet model.
        """
        device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        model = MTCNN(keep_all=True, device=device)
        return model

    def detect_faces(self, image):
        """
        Detects faces in the given image using FaceNet.

        Args:
            image: The input image in which to detect faces.

        Returns:
            List of detected faces.
        """
        # Convert the image to RGB format
        # import pdb; pdb.set_trace()
        self.bridge = CvBridge()
        cv_img = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        image_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        # Detect faces
        face_detected, probs = self.model.detect(image_rgb)
        # boxes, probs, points = self.model.detect(image_rgb, landmarks=True)

        detection_array = Detection2DArray()
        detection_array.header = image.header
        if face_detected is not None:
            for face, prob in zip(face_detected, probs):
                # Create a bounding box for each detected face
                detection = Detection2D()
                detection.header = detection_array.header
                detection.bbox.center.position.x = (face[0] + face[2]) / 2
                detection.bbox.center.position.y = (face[1] + face[3]) / 2
                detection.bbox.center.theta = 0.0
                detection.bbox.size_x = face[2] - face[0]
                detection.bbox.size_y = face[3] - face[1]

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = "face"
                hypothesis.hypothesis.score = prob

                detection.results.append(hypothesis)
                detection_array.detections.append(detection)

                
        else:
            detection_array.detections = []
        
        if self.get_parameter('debug').value:
            # Convert the image back to ROS format and publish it
            debug_image = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            self.debug_publisher_.publish(debug_image)
        
        self.box_publisher_.publish(detection_array)
        

def main():
    rclpy.init()
    face_detection_node = FacenetFaceDetectionNode()
    rclpy.spin(face_detection_node)

if __name__ == '__main__':
    main()