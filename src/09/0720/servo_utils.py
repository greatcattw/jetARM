# servo_utils.py

import time
import math
import sys

from ros_robot_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import ServoPosition

from params_sub6e import (
    servo1_shift, servo2_shift, servo3_shift, servo4_shift, servo5_shift,
    LL, camera_and_claw_y_shift
)


class ServoController:
    def __init__(self, logger, servo_pub):
        self.logger = logger
        self.servo_pub = servo_pub

        # 座標狀態由控制器自己保存
        self.sys_xx = 0.0
        self.sys_yy = 0.0
        self.sys_zz = 0.0
        self.vertical = 1

    def get_xyz(self):
        return self.sys_xx, self.sys_yy, self.sys_zz

    '''
    def set_xyz(self, xx, yy, zz):
        self.sys_xx = float(xx)
        self.sys_yy = float(yy)
        self.sys_zz = float(zz)
    '''
    def ctrl_servos_by_degree(self, duration, positions):
        self.logger.info(f'    duration:{duration}')

        msg = ServosPosition()
        msg.duration = float(duration)
        list_servo_degree = []

        for i1 in positions:
            self.logger.info(f'    {i1[0]} {i1[1]}')
            position = ServoPosition()
            position.id = i1[0]
            position.position = 500 + int(float(i1[1]) / 240 * 1000)

            match position.id:
                case 1:
                    position.position += servo1_shift
                case 2:
                    position.position += servo2_shift
                case 3:
                    position.position += servo3_shift
                case 4:
                    position.position += servo4_shift
                case 5:
                    position.position += servo5_shift

            self.logger.info(f'    servo_id={position.id} pluse={position.position}')
            list_servo_degree.append(position)

        self.logger.info(f'{list_servo_degree}')
        msg.position = list_servo_degree
        self.servo_pub.publish(msg)
        time.sleep(duration + 0.1)
        
    def set_vertical_1_with_inc(self,dx,dy,dz):
        xx0, yy0, zz0 = self.get_xyz()    
        self.vertical = 1
        xx1 = xx0 + dx
        yy1 = yy0 + dy
        zz1 = zz0 + dz        
        move_time = self.move_time(xx0, yy0, zz0, xx1, yy1, zz1)
        if move_time < 1 :
            move_time=1
        self.abs_xyz_to_servo1234(move_time, xx1, yy1, zz1)

    def set_vertical_0_with_inc(self,dx,dy,dz):
        xx0, yy0, zz0 = self.get_xyz()    
        self.vertical = 0
        xx1 = xx0 + dx
        yy1 = yy0 + dy
        zz1 = zz0 + dz        
        move_time = self.move_time(xx0, yy0, zz0, xx1, yy1, zz1)
        if move_time < 1 :
            move_time=1
        self.abs_xyz_to_servo1234(move_time, xx1, yy1, zz1)

    def ctrl_servo_by_value(self, duration, value):
        msg = ServosPosition()
        msg.duration = float(duration) - 0.1

        list_servo_degree = []

        position = ServoPosition()
        position.id = value[0]
        position.position = value[1]

        list_servo_degree.append(position)

        self.logger.info(f'{list_servo_degree}')
        msg.position = list_servo_degree
        self.servo_pub.publish(msg)
        time.sleep(duration)
    def move_time(self,xx0,yy0,zz0,xx1,yy1,zz1):
        move_distance = math.sqrt(
            math.pow((xx0 - xx1), 2) +
            math.pow((yy0 - yy1), 2) +
            math.pow((zz0 - zz1), 2)
        )
        move_time1 = move_distance / 80
        if move_time1 < 0.5:
            move_time = 0.5
        return move_time1

    def abs_xyz_to_servo1234(self, time_to_move, f_abs_xx, f_abs_yy, f_abs_zz):
        flg_in_range = True

        x2 = LL + f_abs_xx
        y2 = f_abs_yy
        z2 = LL + f_abs_zz

        ll = math.sqrt(math.pow(x2, 2) + math.pow(y2, 2) + math.pow(z2, 2))
        self.logger.info(f'll={ll}')

        if ll > (2 * LL):
            self.logger.info(f'Error, too far, out of range of arm {ll} ')
            time.sleep(1)
            sys.exit(1)
            flg_in_range = False

        x2y2ll = math.sqrt(math.pow(x2, 2) + math.pow(y2, 2))
        A5a = math.degrees(math.acos(x2y2ll / ll))
        A5b = math.degrees(math.acos((ll / 2) / LL))
        A5 = -90 + (A5a + A5b)
        self.logger.info(f'A5a={round(A5a, 3)} A5b={round(A5b, 3)} A5={round(A5, 3)}')

        A4 = -90 + (90 - 2 * (A5b))
        self.logger.info(f'A4={round(A4, 3)}')

        if self.vertical == 1:
            A3 = -90 - (A5a - A5b)
        else:
            A3 = 0 - (A5a - A5b)
        self.logger.info(f'A3={round(A3, 3)}')

        # 比 atan(y/x) 更安全
        A1 = math.degrees(math.atan2(y2, x2))
        self.logger.info(f'A1={round(A1, 3)}')

        if flg_in_range:
            self.sys_xx = f_abs_xx
            self.sys_yy = f_abs_yy
            self.sys_zz = f_abs_zz
            self.ctrl_servos_by_degree(time_to_move, [[1, A1], [2, A5], [3, A4], [4, A3]])
            
            

