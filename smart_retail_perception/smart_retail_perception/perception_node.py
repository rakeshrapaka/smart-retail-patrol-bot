import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def listener_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        boxes, _ = self.hog.detectMultiScale(cv_image, winStride=(8,8))
        for (x, y, w, h) in boxes:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        if len(boxes) > 0:
            self.get_logger().info(f"Humans detected: {len(boxes)}")
        cv2.imshow("Human Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HumanDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
