# sub6.py

import rclpy
import time
import math

from rclpy.node import Node
from std_msgs.msg import String
from ros_robot_controller_msgs.msg import ServosPosition

from params_sub6e import (
    LL, 
    camera_and_claw_y_shift,
    camera_and_claw_x_shift,
    claw_open, claw_3cm, claw_close, claw_bottle,
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
            20
        )

        self.get_logger().info('    wait ready')
        time.sleep(3)

        # to ucs 0,0,0
        self.servo_ctrl.ctrl_servos_by_degree(
            4,
            [[1, 0], [2, 0], [3, -90], [4, -90], [5, 0], [10, (claw_open - 500) * 240 / 1000]]
        )

        self.get_logger().info('    ok')
        
    def xy_plane_wrist_inc_radius(self,xx1,yy1,inc_rr):
        now_xy_degree1 = math.atan2(yy1, (xx1 + LL))
        xx2 = xx1 + math.cos(now_xy_degree1) * inc_rr
        yy2 = yy1 + math.sin(now_xy_degree1) * inc_rr
        return xx2,yy2 

    def listener_callback(self, msg):
        data = msg.data.split()
        id_op = int(data[0])
        float_values = [float(x) for x in data[1:]]
        self.get_logger().info(f'Received: Integer: {id_op}, Floats: {float_values}')

        match id_op:
            case 0:  # abs move , claw_vertical
                now_xx, now_yy, now_zz = self.servo_ctrl.get_xyz()
                new_xx = float(data[1])
                new_yy = float(data[2])
                new_zz = float(data[3])
                move_time = self.servo_ctrl.move_time(now_xx, now_yy, now_zz,new_xx, new_yy, new_zz)
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
                self.servo_ctrl.ctrl_servos_by_degree(0.9, [[5, float(data[1])]])
            case 10:
                self.servo_ctrl.ctrl_servos_by_degree(0.9, [[10, float(data[1])]])

            case 11:
                self.servo_ctrl.ctrl_servo_by_value(0.9, [10, claw_open])
            case 12:
                self.servo_ctrl.ctrl_servo_by_value(0.9, [10, claw_3cm])
            case 13:
                self.servo_ctrl.ctrl_servo_by_value(0.9, [10, claw_close])
            case 14:
                self.servo_ctrl.ctrl_servo_by_value(0.9, [10, claw_bottle])                

            case 20:  # wrist inc move
                now_xx, now_yy, now_zz = self.servo_ctrl.get_xyz()

                dx = float(data[1])
                dy = float(data[2])
                dz = float(data[3])

                new_xx = now_xx + dx
                new_yy = now_yy + dy
                new_zz = now_zz + dz

                move_time = self.servo_ctrl.move_time(now_xx, now_yy, now_zz,new_xx, new_yy, new_zz)
                #s elf.get_logger().info(f'move={round(move_distance, 3)}, time={round(move_time, 3)}')

                #self.servo_ctrl.set_xyz(new_xx, new_yy, new_zz)
                self.servo_ctrl.abs_xyz_to_servo1234(move_time, new_xx, new_yy, new_zz)

            case 30:  #wrist xy_plane_polar_coordination_inc_rr
                
                now_xx, now_yy, now_zz = self.servo_ctrl.get_xyz()
                '''
                now_xy_degree = math.atan2(now_yy, (now_xx + LL))
                add_radius = float(data[1])

                new_xx = now_xx + math.cos(now_xy_degree) * add_radius
                new_yy = now_yy + math.sin(now_xy_degree) * add_radius
                ''' 
                add_radius = float(data[1])
                now_xy_degree = math.atan2(now_yy, (now_xx + LL))
                new_xx,new_yy = self.xy_plane_wrist_inc_radius(now_xx, now_yy,add_radius)
                
                #self.servo_ctrl.set_xyz(new_xx, new_yy, now_zz)
                self.servo_ctrl.abs_xyz_to_servo1234(1, new_xx, new_yy, now_zz)

                self.get_logger().info(
                    f'x={round(new_xx, 3)} y={round(new_yy, 3)} degree={round(math.degrees(now_xy_degree), 3)}'
                )
                
            case 40:
                now_xx, now_yy, now_zz = self.servo_ctrl.get_xyz()
                now_xy_degree = math.atan2(now_yy, (now_xx + LL))
                now_xy_ll = math.sqrt(math.pow(now_xx + LL, 2) + math.pow(now_yy, 2))
                add_cw1 = 2 * math.asin((float(camera_and_claw_x_shift)/2) / (now_xy_ll))

                new_xx = now_xy_ll * math.cos(now_xy_degree + add_cw1) - LL
                new_yy = now_xy_ll * math.sin(now_xy_degree + add_cw1)

                self.get_logger().info(f'now_xy=({now_xx},{now_yy}) new_xy({new_xx},{new_yy})')
                self.get_logger().info(
                    f'll={round(now_xy_ll, 3)} now_d={round(math.degrees(now_xy_degree),3)} add_d={round(math.degrees(add_cw1),3)}'
                )
                self.servo_ctrl.ctrl_servos_by_degree(0.2, [[5, float(data[1])+math.degrees(add_cw1)]])
                #self.get_logger().info('5555555555555555555555555555555555555555555555555')
                time.sleep(0.5)
                #self.get_logger().info('7777777777777777777777777777777777777777777777777')
                #self.servo_ctrl.set_xyz(new_xx, new_yy, now_zz)
                self.servo_ctrl.abs_xyz_to_servo1234(0.5, new_xx, new_yy, now_zz)
            case 50: #img_add_x_y
                now_xx, now_yy, now_zz = self.servo_ctrl.get_xyz()
                now_xy_degree = math.atan2(now_yy, (now_xx + LL))

                # camera_claw_shift=47
                xx1 = now_xx + math.cos(now_xy_degree) * camera_and_claw_y_shift
                yy1 = now_yy + math.sin(now_xy_degree) * camera_and_claw_y_shift
                xx1yy1_degree = now_xy_degree
                xx1yy1_ll = math.sqrt(math.pow(xx1 + LL, 2) + math.pow(yy1, 2))
                self.get_logger().info(f'xx1yy1= x={round(xx1,3)} , y={round(yy1,3)}')
                self.get_logger().info(f'xx1yy1= r={round(xx1yy1_ll,3)} , d={round(math.degrees(xx1yy1_degree),3)}')
                ### 
                
  
                add_cw1 = 2 * math.asin((camera_and_claw_x_shift/2) / (xx1yy1_ll))       
                # (xx2,yy2) now img center of wrist
                xx2 = xx1yy1_ll * math.cos(xx1yy1_degree + add_cw1) - LL
                yy2 = xx1yy1_ll * math.sin(xx1yy1_degree + add_cw1)
          
                self.get_logger().info(f'xx2yy2= x={round(xx2,3)} , y={round(yy2,3)}')
                #self.get_logger().info(f'xx2yy2={round(xx2yy2_ll,3)} , {round(math.degrees(xx2yy2_degree),3)}')
                
                ###
                # (xx3,yy3)new img center of wrist
                xx3 = xx2 + float(data[1])
                yy3 = yy2 + float(data[2])
                xx3yy3_degree = math.atan2(yy3, (xx3 + LL))
                xx3yy3_ll= math.sqrt(math.pow(xx3 + LL, 2) + math.pow(yy3, 2))
                self.get_logger().info(f'xx3yy3= x={round(xx3,3)} , y={round(yy3,3)}')
                self.get_logger().info(f'xx3yy3= r={round(xx3yy3_ll,3)} , d={round(math.degrees(xx3yy3_degree),3)}')
                
                ###
                dec_cw1 = 2 * math.asin((camera_and_claw_x_shift/2) / (xx3yy3_ll)) 
                xx4yy4_degree=xx3yy3_degree-dec_cw1
                xx4 = xx3yy3_ll * math.cos(xx4yy4_degree) -LL
                yy4 = xx3yy3_ll * math.sin(xx4yy4_degree)
                xx4yy4_ll=xx3yy3_ll
                self.get_logger().info(f'xx4yy4= x={round(xx4,3)} , y={round(yy4,3)}')
                self.get_logger().info(f'xx4yy4= r={round(xx4yy4_ll,3)} , d={round(math.degrees(xx4yy4_degree),3)}')

                ###
                xx5yy5_degree=xx4yy4_degree
                xx5 = xx4 - math.cos(xx5yy5_degree) * camera_and_claw_y_shift
                yy5 = yy4 - math.sin(xx5yy5_degree) * camera_and_claw_y_shift
                move_time = self.servo_ctrl.move_time(now_xx, now_yy, now_zz,xx5, yy5, now_zz)              
                self.get_logger().info(f'xx5yy5= x={round(xx5,3)} , y={round(yy5,3)}')
                self.servo_ctrl.abs_xyz_to_servo1234(move_time, xx5, yy5, now_zz)               
            case 60: #servo#4, claw horizental, to xy   
                dx = float(data[1])
                dy = float(data[2])
                dz = float(data[3])            
                self.servo_ctrl.set_vertical_0_with_inc(dx,dy,dz)
            case 61: #servo#4, claw vertical, to z-   
                dx = float(data[1])
                dy = float(data[2])
                dz = float(data[3])            
                self.servo_ctrl.set_vertical_1_with_inc(dx,dy,dz)

            case 70: #servo#5, when claw to z-,  parallel +y -y
                now_xx, now_yy, now_zz = self.servo_ctrl.get_xyz()
                now_xy_degree = math.atan2(now_yy, (now_xx + LL))      
                st1 = f'x={round(now_xx,3)},y={round(now_yy,3)} '
                st1 = st1 + f'd={round(math.degrees(now_xy_degree),3)}'
                self.get_logger().info(st1)
                self.servo_ctrl.ctrl_servos_by_degree(0.5, [[5, math.degrees(now_xy_degree)]])        
            case 80: #move wrist to view-center dx dy dz 
                xx0, yy0, zz0 = self.servo_ctrl.get_xyz()    
                dx = float(data[1])
                dy = float(data[2])
                dz = float(data[3]) 
                degree0 = math.atan2(yy0, (xx0 + LL)) 
                add_radius = camera_and_claw_y_shift 
                
                xx1 = xx0 + math.cos(degree0) * add_radius
                yy1 = yy0 + math.sin(degree0) * add_radius   
                degree1 = degree0
                rr1 = math.sqrt(math.pow(xx1 + LL, 2) + math.pow(yy1, 2))
                
                add_cw1 = 2 * math.asin((float(camera_and_claw_x_shift)/2) / (rr1)) 
                xx2 = rr1 * math.cos(degree1 + add_cw1) - LL
                yy2 = rr1 * math.sin(degree1 + add_cw1) 

                xx3 = xx2 + dx
                yy3 = yy2 + dy
                zz3 = zz0 + dz                                    
                move_time = self.servo_ctrl.move_time(xx0, yy0, zz0, xx3, yy3, zz3)                 
                self.get_logger().info(f'cmd80 x={round(xx3,3)} , y={round(yy3,3)} , z={round(zz3,3)}')
                self.servo_ctrl.abs_xyz_to_servo1234(move_time, xx3, yy3, zz3)  
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
