#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import time


class SingleImageSaverViewer(Node):
    def __init__(self):
        super().__init__('single_image_saver_viewer')

        self.bridge = CvBridge()
        self.image_saved = False
        self.running = True

        self.subscription = self.create_subscription(
            Image,
            '/depth_cam/rgb/image_raw',
            self.image_callback,
            10
        )

        self.window_name = 'Captured Image'
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        self.get_logger().info('Subscribed to /depth_cam/rgb/image_raw')
        self.get_logger().info('Waiting for one image...')
        self.get_logger().info('Press any key to quit.')

    def image_callback(self, msg):
        if self.image_saved:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # 存成 1.jpg
        cv2.imwrite('1.jpg', frame)
        self.get_logger().info('Image saved as 1.jpg')

        # 顯示影像
        cv2.imshow(self.window_name, frame)
        self.image_saved = True

        # 等待任意鍵
        if len(sys.argv) != 2:
            key = cv2.waitKey(0)
            if key != -1:
                self.image_saved = True
                self.get_logger().info('Quit requested by user.')
                self.running = False
        else:
            self.running = False

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SingleImageSaverViewer()

    try:
        while rclpy.ok() and node.running and not node.image_saved:
            rclpy.spin_once(node, timeout_sec=0.1)

        # 若已顯示影像，這裡再保持節點運作直到按鍵結束
        while rclpy.ok() and node.running and node.image_saved:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received.')

    finally:
        node.running = False
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
