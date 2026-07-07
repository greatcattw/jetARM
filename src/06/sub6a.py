# sub6.py

import rclpy
import time
import math

from rclpy.node import Node
from std_msgs.msg import String
from ros_robot_controller_msgs.msg import ServosPosition

from params import (
    LL, camera_and_claw_y_shift,
    claw_open, claw_3cm, claw_close
)
from servo_utils import ServoController


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        self.servo_pub = self.create_publisher(
            ServosPosition,
            '/ros_robot_controller/bus_servo/set_position',
            1
        )
        self.get_logger().info('    create_publisher servo_pub')

        self.servo_ctrl = ServoController(
            logger=self.get_logger(),
            servo_pub=self.servo_pub
        )

        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10
        )

        self.get_logger().info('    wait ready')
        time.sleep(3)

        # to ucs 0,0,0
        self.servo_ctrl.ctrl_servos_by_degree(
            4,
            [[1, 0], [2, 0], [3, -90], [4, -90], [5, 0], [10, (claw_open - 500) * 240 / 1000]]
        )

        self.get_logger().info('    ok')

    def listener_callback(self, msg):
        data = msg.data.split()
        id_op = int(data[0])
        float_values = [float(x) for x in data[1:]]
        self.get_logger().info(f'Received: Integer: {id_op}, Floats: {float_values}')

        match id_op:
            case 0:  # abs move
                now_xx, now_yy, now_zz = self.servo_ctrl.get_xyz()

                move_distance = math.sqrt(
                    math.pow(now_xx - float(data[1]), 2) +
                    math.pow(now_yy - float(data[2]), 2) +
                    math.pow(now_zz - float(data[3]), 2)
                )
                move_time = move_distance / 80
                if move_time < 0.2:
                    move_time = 0.2

                self.get_logger().info(f'move={round(move_distance, 3)}, time={round(move_time, 3)}')

                new_xx = float(data[1])
                new_yy = float(data[2])
                new_zz = float(data[3])

                self.servo_ctrl.set_xyz(new_xx, new_yy, new_zz)
                self.servo_ctrl.abs_xyz_to_servo1234(move_time, new_xx, new_yy, new_zz)

            case 1:
                self.servo_ctrl.ctrl_servos_by_degree(1, [[1, float(data[1])]])
            case 2:
                self.servo_ctrl.ctrl_servos_by_degree(1, [[2, float(data[1])]])
            case 3:
                self.servo_ctrl.ctrl_servos_by_degree(1, [[3, float(data[1])]])
            case 4:
                self.servo_ctrl.ctrl_servos_by_degree(1, [[4, float(data[1])]])
            case 5:
                self.servo_ctrl.ctrl_servos_by_degree(1, [[5, float(data[1])]])
            case 10:
                self.servo_ctrl.ctrl_servos_by_degree(1, [[10, float(data[1])]])

            case 11:
                self.servo_ctrl.ctrl_servo_by_value(1, [10, claw_open])
            case 12:
                self.servo_ctrl.ctrl_servo_by_value(1, [10, claw_3cm])
            case 13:
                self.servo_ctrl.ctrl_servo_by_value(1, [10, claw_close])

            case 20:  # inc move
                now_xx, now_yy, now_zz = self.servo_ctrl.get_xyz()

                dx = float(data[1])
                dy = float(data[2])
                dz = float(data[3])

                new_xx = now_xx + dx
                new_yy = now_yy + dy
                new_zz = now_zz + dz

                move_distance = math.sqrt(
                    math.pow(dx, 2) +
                    math.pow(dy, 2) +
                    math.pow(dz, 2)
                )
                move_time = move_distance / 80
                if move_time < 0.2:
                    move_time = 0.2

                self.get_logger().info(f'move={round(move_distance, 3)}, time={round(move_time, 3)}')

                self.servo_ctrl.set_xyz(new_xx, new_yy, new_zz)
                self.servo_ctrl.abs_xyz_to_servo1234(move_time, new_xx, new_yy, new_zz)

            case 30:  # xy polar move
                now_xx, now_yy, now_zz = self.servo_ctrl.get_xyz()
                now_xy_degree = math.atan2(now_yy, (now_xx + LL))
                add_radius = float(data[1])

                new_xx = now_xx + math.cos(now_xy_degree) * add_radius
                new_yy = now_yy + math.sin(now_xy_degree) * add_radius

                self.servo_ctrl.set_xyz(new_xx, new_yy, now_zz)
                self.servo_ctrl.abs_xyz_to_servo1234(1, new_xx, new_yy, now_zz)

                self.get_logger().info(
                    f'x={round(new_xx, 3)} y={round(new_yy, 3)} degree={round(math.degrees(now_xy_degree), 3)}'
                )

            case 40:
                now_xx, now_yy, now_zz = self.servo_ctrl.get_xyz()
                now_xy_degree = math.atan2(now_yy, (now_xx + LL))
                now_xy_ll = math.sqrt(math.pow(now_xx + LL, 2) + math.pow(now_yy, 2))
                add_cw1 = 2 * math.asin(10 / (now_xy_ll + camera_and_claw_y_shift))

                new_xx = now_xy_ll * math.cos(now_xy_degree + add_cw1) - LL
                new_yy = now_xy_ll * math.sin(now_xy_degree + add_cw1)

                self.get_logger().info(f'now_xy=({now_xx},{now_yy}) new_xy({new_xx},{new_yy})')
                self.get_logger().info(
                    f'll={round(now_xy_ll, 3)} now_d={round(now_xy_degree, 3)} add_d={round(add_cw1, 3)}'
                )

                self.servo_ctrl.set_xyz(new_xx, new_yy, now_zz)
                self.servo_ctrl.abs_xyz_to_servo1234(0.5, new_xx, new_yy, now_zz)

            case 100:
                self.servo_ctrl.ctrl_servos_by_degree(
                    4,
                    [[1, 0], [2, 0], [3, 0], [4, 0], [5, 0]]
                )


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
