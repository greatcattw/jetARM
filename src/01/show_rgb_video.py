#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ImageCenterCrossViewer(Node):
    def __init__(self):
        super().__init__('image_center_cross_viewer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/depth_cam/rgb/image_raw',
            self.image_callback,
            10
        )

        self.window_name = 'RGB Image 1:1'
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        self.get_logger().info('Subscribed to /depth_cam/rgb/image_raw')

    def image_callback(self, msg):
        try:
            # 保持原始影像尺寸，不做縮放
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2

        # 畫垂直中心紅線
        cv2.line(
            frame,
            (center_x, 0),
            (center_x, height - 1),
            (0, 0, 255),
            1
        )

        # 畫水平中心紅線
        cv2.line(
            frame,
            (0, center_y),
            (width - 1, center_y),
            (0, 0, 255),
            1
        )

        # 1:1 顯示，不手動 resize
        cv2.imshow(self.window_name, frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            self.get_logger().info('Quit requested by user.')
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageCenterCrossViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
