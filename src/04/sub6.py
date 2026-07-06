import rclpy
import time
import math
import sys
from rclpy.node import Node
from std_msgs.msg import String
from ros_robot_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import ServoPosition

servo1_shift=0
servo2_shift=10
servo3_shift=0
servo4_shift=-5
servo5_shift=0
LL=int(129) # up_arm=front_arm=129 mm
camera_and_claw_y_shift=float(47)
claw_open=100 #cmd 90 500 + int(float(i1[1])/240*1000)
claw_3cm=516 #cmd 91
claw_close=650 #cmd 92


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.servo_pub = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 1)   
        self.get_logger().info('    create_publisher servo_pub') 
               
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.get_logger().info('    wait ready')    
        time.sleep(3)    
        # to ucs 0,0,0
        self.ctrl_servos_by_degree(4,[[1,0],[2,0],[3,-90],[4,-90],[5,0],[10,(claw_open-500)*240/1000]])
        self.sys_xx=float(0)
        self.sys_yy=float(0)
        self.sys_zz=float(0)
        self.get_logger().info('    ok') 


    def listener_callback(self, msg):
        data = msg.data.split()
        id_op = int(data[0])
        float_values = [float(x) for x in data[1:]]
        self.get_logger().info(f'Received: Integer: {id_op}, Floats: {float_values}')
        match id_op:
            case 0: # asb move
                move_distance=math.sqrt(math.pow(self.sys_xx-float(data[1]),2)+math.pow(self.sys_yy-float(data[2]),2) +math.pow(self.sys_zz-float(data[3]),2))
                move_time=move_distance / 80 # 1 sec move 80mm
                if move_time < 0.2 :
                    move_time = 0.2;                
                self.get_logger().info(f'move={round(move_distance,3)}, time={round(move_time,3)}')             
                self.sys_xx =float(data[1])
                self.sys_yy =float(data[2])
                self.sys_zz =float(data[3])
                self.abs_xyz_to_servo1234(move_time,self.sys_xx,self.sys_yy,self.sys_zz)
            case 1:
                self.ctrl_servos_by_degree(1,[[1,float(data[1])]])   
            case 2:
                self.ctrl_servos_by_degree(1,[[2,float(data[1])]])   
            case 3:
                self.ctrl_servos_by_degree(1,[[3,float(data[1])]])   
            case 4:
                self.ctrl_servos_by_degree(1,[[4,float(data[1])]])   
            case 5:
                self.ctrl_servos_by_degree(1,[[5,float(data[1])]])   
            case 10:
                self.ctrl_servos_by_degree(1,[[10,float(data[1])]])                
            case 11:
                self.ctrl_servo_by_value(1,[10,claw_open])    
            case 12:
                self.ctrl_servo_by_value(1,[10,claw_3cm])
            case 13:
                self.ctrl_servo_by_value(1,[10,claw_close])
                
            case 20: # inc move
                self.sys_xx +=float(data[1])
                self.sys_yy +=float(data[2])
                self.sys_zz +=float(data[3])
                move_distance=math.sqrt(math.pow(float(data[1]),2)+math.pow(float(data[2]),2) +math.pow(float(data[3]),2))
                move_time=move_distance / 80 # 1 sec move 80mm
                if move_time < 0.2 :
                    move_time = 0.2;                
                self.get_logger().info(f'move={round(move_distance,3)}, time={round(move_time,3)}')  
                self.abs_xyz_to_servo1234(move_time,self.sys_xx,self.sys_yy,self.sys_zz)
            case 30: # xy polar move
                now_xy_degree=math.atan(self.sys_yy/(self.sys_xx+LL))
                add_radius=float(data[1])
                self.sys_xx +=math.cos(now_xy_degree) * add_radius
                self.sys_yy +=math.sin(now_xy_degree) * add_radius           
                self.abs_xyz_to_servo1234(1,self.sys_xx,self.sys_yy,self.sys_zz)
                self.get_logger().info(f'x={round(self.sys_xx,3)} y={round(self.sys_yy,3)} degree={round(math.degrees(now_xy_degree),3)}')
            case 40:
                now_xy_degree=math.atan(self.sys_yy/(self.sys_xx+LL))            
                now_xy_ll=math.sqrt(math.pow(self.sys_xx+LL,2)+math.pow(self.sys_yy,2))                
                add_cw1=2*math.asin(10/(now_xy_ll+camera_and_claw_y_shift))
                new_xx=now_xy_ll*math.cos(now_xy_degree+add_cw1) - LL
                new_yy=now_xy_ll*math.sin(now_xy_degree+add_cw1)
                self.get_logger().info(f'now_xy=({self.sys_xx},{self.sys_yy}) new_xy({new_xx},{new_yy})')
                self.get_logger().info(f'll={round(now_xy_ll,3)} now_d={round(now_xy_degree,3)} add_d={round(add_cw1,3)}' )
                self.sys_xx=new_xx
                self.sys_yy=new_yy
                self.abs_xyz_to_servo1234(0.5,self.sys_xx,self.sys_yy,self.sys_zz)

            case 100:
                self.ctrl_servos_by_degree(4,[[1,0],[2,0],[3,0],[4,0],[5,0]])
            
    def abs_xyz_to_servo1234(self,time_to_move,f_abs_xx,f_abs_yy,f_abs_zz):
        flg_in_range=True
        #self.get_logger().info(f'{f_abs_xx},{f_abs_yy},{f_abs_zz}')
        
        x2=LL+f_abs_xx
        y2=f_abs_yy
        z2=LL+f_abs_zz
        ll=math.sqrt(math.pow(x2,2)+math.pow(y2,2)+math.pow(z2,2))
        self.get_logger().info(f'll={ll}')        
        if ll > (2 * LL):
            self.get_logger().info(f'Error, too far, out of range of arm {ll} ')
            time.sleep(1)
            sys.exit(1)
            flg_in_range=False
        x2y2ll=math.sqrt(math.pow(x2,2)+math.pow(y2,2))    
        A5a=math.degrees(math.acos(x2y2ll/ll))
        A5b=math.degrees(math.acos((ll/2)/LL))
        A5=-90+(A5a+A5b)
        self.get_logger().info(f'A5a={round(A5a,3)} A5b={round(A5b,3)} A5={round(A5,3)}')
        A4=-90+(90-2*(A5b))
        self.get_logger().info(f'A4={round(A4,3)}')
        A3=-90-(A5a-A5b)
        self.get_logger().info(f'A3={round(A3,3)}')
        
        A1=math.degrees(math.atan(y2/x2))
        self.get_logger().info(f'A1={round(A1,3)}')
        
        #move_distance=math.sqrt(math.pow(f_abs_xx,2)+math.pow(f_abs_yy,2)+math.pow(f_abs_zz,2))
        #move_time=move_distance / 50 # 1 sec move 50mm
        #if move_time < 1.0 :
        #  move_time = 1.0;
        #self.get_logger().info(f'move={round(move_distance,3)}, time={round(move_time,3)}')  
        if flg_in_range :
            self.ctrl_servos_by_degree(time_to_move,[[1,A1],[2,A5],[3,A4],[4,A3]])  
        
    def ctrl_servo_by_value(self,duration, value):
        msg = ServosPosition()
        msg.duration = float(duration)-0.1     
        list_servo_degree = []
            
        position = ServoPosition() # init object
        position.id =value[0]
        position.position =value[1]

        list_servo_degree.append(position)
        
        self.get_logger().info(f'{list_servo_degree}')
        msg.position = list_servo_degree    
        self.servo_pub.publish(msg)
        time.sleep(duration)          

    def ctrl_servos_by_degree(self,duration, positions):
        # ref llm_control_servo_offline.py / def set_position(self, duration, positions):
        self.get_logger().info(f'    d:{duration}')

        msg = ServosPosition()
        msg.duration = float(duration)     
        list_servo_degree = []
        
        for i1 in positions:
            self.get_logger().info(f'    {i1[0]} {i1[1]}')
            position = ServoPosition() # init object
            position.id = i1[0]
            position.position = 500 + int(float(i1[1])/240*1000)
            match position.id:
                case 1:
                    position.position +=servo1_shift            
                case 2:
                    position.position +=servo2_shift
                case 3:
                    position.position +=servo3_shift
                case 4:
                    position.position +=servo4_shift
                case 5:
                    position.position +=servo5_shift
            
            self.get_logger().info(f'    servo_id={position.id} pluse={position.position}')
            list_servo_degree.append(position)
        self.get_logger().info(f'{list_servo_degree}')
        msg.position = list_servo_degree    
        self.servo_pub.publish(msg)
        time.sleep(duration+0.1)         
        

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
