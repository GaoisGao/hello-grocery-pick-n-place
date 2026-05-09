import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.get_logger().info('QR detector node up')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        data, points, _ = self.detector.detectAndDecode(cv_image)

        if points is not None and data:
            self.get_logger().info(f'Detected QR: "{data}"')

            # draw the detected polygon
            pts = points[0].astype(int)
            cv2.polylines(cv_image, [pts], True, (0, 255, 0), 3)

            # label with the decoded text near the top-left corner
            cv2.putText(cv_image, data, tuple(pts[0]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # mark each corner with a small circle so you can see ordering
            for i, pt in enumerate(pts):
                cv2.circle(cv_image, tuple(pt), 6, (0, 0, 255), -1)
                cv2.putText(cv_image, str(i), tuple(pt + 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow('QR Detector', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



