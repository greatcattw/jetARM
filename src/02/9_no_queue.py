#!/usr/bin/python3
# coding=utf8

import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class ShowDepthOnlyDirectNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.running = True

        # 深度有效範圍，單位通常為 mm
        self.min_depth = 100
        self.max_depth = 500

        # 第一次收到影像時，印出影像尺寸
        self.image_info_logged = False

        # OpenCV 視窗名稱
        self.window_name = "depth_250_to_500"

        # 使用 WINDOW_AUTOSIZE：
        # 視窗大小會依照影像原始大小顯示，達到 1:1 顯示效果
        cv2.namedWindow("depth_100_to_500", cv2.WINDOW_AUTOSIZE)
        
        # 關閉 LDP，參考原始程式設定
        self.client = self.create_client(SetBool, '/depth_cam/set_ldp_enable')
        self.client.wait_for_service()

        # 只訂閱深度影像
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_cam/depth/image_raw',
            self.depth_callback,
            1
        )

        timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            0.0,
            self.init_process,
            callback_group=timer_cb_group
        )

    def init_process(self):
        self.timer.cancel()

        # 關閉 LDP
        msg = SetBool.Request()
        msg.data = False
        self.send_request(self.client, msg)

        self.get_logger().info('\033[1;32m%s\033[0m' % 'show depth only direct start')

    def send_request(self, client, msg):
        future = client.call_async(msg)

        while rclpy.ok():
            if future.done():
                return future.result()

    def depth_callback(self, ros_depth_image):
        try:
            # ROS Image 轉 numpy depth image
            depth_image = np.ndarray(
                shape=(ros_depth_image.height, ros_depth_image.width),
                dtype=np.uint16,
                buffer=ros_depth_image.data
            )

            # 複製一份，避免直接操作 ROS message buffer
            depth_image = np.copy(depth_image)

            h, w = depth_image.shape[:2]

            # 第一次收到影像時，印出影像尺寸
            if not self.image_info_logged:
                self.get_logger().info(
                    'depth image size: width = %d, height = %d' % (w, h)
                )
                self.image_info_logged = True

            # 有效深度範圍：250～500 mm
            valid_mask = (depth_image >= self.min_depth) & (depth_image <= self.max_depth)

            # 將深度值裁剪到 250～500
            clipped_depth = np.clip(depth_image, self.min_depth, self.max_depth)

            # 將 250～500 mm 映射到 0～255
            #
            # 250 mm -> 0
            # 500 mm -> 255
            depth_8bit = (
                (clipped_depth.astype(np.float32) - self.min_depth)
                / (self.max_depth - self.min_depth)
                * 255.0
            ).astype(np.uint8)
            
            depth_8bit = 255 - depth_8bit            

            # 轉成?彩色
            depth_color_map = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

            # 距離範圍之外的點顯示黑色
            # 也就是 depth < 250 或 depth > 500 的位置
            depth_color_map[~valid_mask] = [0, 0, 0]

            # 畫出中心紅色水平線與垂直線
            center_x = w // 2
            center_y = h // 2

            red_color = (0, 0, 255)  # OpenCV 使用 BGR，紅色為 (0, 0, 255)
            line_thickness = 1

            cv2.line(
                depth_color_map,
                (0, center_y),
                (w - 1, center_y),
                red_color,
                line_thickness
            )

            cv2.line(
                depth_color_map,
                (center_x, 0),
                (center_x, h - 1),
                red_color,
                line_thickness
            )

            # 1:1 顯示深度?彩圖
            # 因為 namedWindow 使用 WINDOW_AUTOSIZE，
            # 所以 imshow 會依照影像原始尺寸顯示，不會任意縮放
            cv2.imshow("depth_100_to_500", depth_color_map)
            
            key = cv2.waitKey(1)

            # 按任意鍵結束
            if key != -1:
                self.running = False
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().info('error: ' + str(e))


def main():
    node = ShowDepthOnlyDirectNode('show_depth_only_direct')

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.running = False
    cv2.destroyAllWindows()
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
