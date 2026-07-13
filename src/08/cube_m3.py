import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from dt_apriltags import Detector
from gcat_utils import Gcat_utils
from ros_robot_controller_msgs.msg import BuzzerState


from cube_m_params import (
    pixel_tolerence,
    pixels_per_30mm,
    pixel_move_long_or_short,
    move_long,
    move_short,
)


class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.flg1=True
        self.flg_z_m_40=False
        self.flg_z_m_40_done=False
        self.f_atag_pm45=float(0)
        self.gcat_utils = Gcat_utils(
            logger=self.get_logger()
        )        
        #########
        self.image_saved = False
        self.bridge = CvBridge()
        self.image = None
        self.subscription = self.create_subscription(
            Image,
            '/depth_cam/rgb/image_raw',
            self.image_callback,
            1
        )        
        self.window_name = 'Captured Image'
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        #######
        self.pub_buzzer = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer',1)
        self.msg_c5 = BuzzerState(freq = 525, on_time = 0.01, off_time = 0.01, repeat = 1)
        self.msg_c5L = BuzzerState(freq = 525, on_time = 0.1, off_time = 0.01, repeat = 1)
        self.msg_c4 = BuzzerState(freq = 263, on_time = 0.01, off_time = 0.01, repeat = 1)
        self.msg_err = BuzzerState(freq = 1000, on_time = 0.1, off_time = 0.1, repeat = 3)
        #######        
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)
        self.get_logger().info(f'    wait create_publisher')
        #self.timer=self.create_timer(3.0,self.timer_callback)
        time.sleep(3)
        self.get_logger().info(f'    ready')
        self.i1=0
        



    def track_atag(self):
        #if  (self.i1<3):
            self.i1 +=1
            self.get_logger().info(f'    {self.i1}')   
            
            img_height, img_width = self.image.shape[:2]
            i_img_center_yy = int(img_height / 2)
            i_img_center_xx = int(img_width / 2 )
            self.get_logger().info(f'img_center=({i_img_center_xx},{i_img_center_yy})')

            # 灰階轉換
            gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

            # 建立 AprilTag 偵測器
            detector = Detector(
                searchpath=['apriltags'],
                families='tag36h11',
                nthreads=8,
                quad_decimate=2.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0
            )

            # 偵測 AprilTag
            tags = detector.detect(gray, False, None, 0.025)            
            
            if len(tags) == 0:
                print("圖片中沒有偵測到 AprilTag。")
                cv2.imshow(self.window_name, self.image)     
                self.pub_buzzer.publish(self.msg_c5)                 
                key = cv2.waitKey(300)
                self.pub_buzzer.publish(self.msg_c4)
                key = cv2.waitKey(0)                
                self.flg1=False
        
            else:
                # 只有一個 tag，直接取第一個
                tag = tags[0]

                at_center_x, at_center_y = tag.center.astype(int)
                print(f"偵測到 AprilTag ID: {tag.tag_id}")
                print(f"AT中心點座標: ({at_center_x}, {at_center_y})")

                # 標示 tag
                self.gcat_utils.draw_tag(self.image, tag,pixel_tolerence,pixel_move_long_or_short)
                #cv2.imshow(self.window_name, self.image) 
                #cv2.waitKey(1)

                #high z and try to lock atag -> low z and try to lock atag -> move_cube
                
                st1=self.gcat_utils.plan_arm_action(at_center_x,at_center_y,i_img_center_xx,i_img_center_yy,tag) 
                #st_argv_for_arm=f"20 {st1} 0"
                st_argv_for_arm=st1
                self.get_logger().info(f'st_argv_for_arm={st_argv_for_arm}')
                if not st_argv_for_arm=="20 0 0 0": #try to lock atag
                    self.publisher_.publish(String(data=st_argv_for_arm))  
                else: # ok for locking atag
                    if not self.flg_z_m_40: # ok for locking atag with high z
                        self.publisher_.publish(String(data="20 0 0 -40")) # z -40
                        self.flg_z_m_40=True
                        #self.get_logger().info('111111111111')                              
                    else: # ok for locking atag with low z
                        self.f_atag_pm45=self.gcat_utils.cal_atag_pm45(tag.corners)
                        self.flg_z_m_40_done=True

                if st_argv_for_arm=="20 0 0 0" and self.flg_z_m_40_done :
                    # ok for locking atag 
                    self.pub_buzzer.publish(self.msg_c5L)
                    self.move_cube()
                else:
                    self.pub_buzzer.publish(self.msg_c4) 
                    cv2.imshow(self.window_name, self.image) 
                    if len(sys.argv) == 2:
                        cv2.waitKey(0)
                    else:
                        cv2.waitKey(1000)                                                     
                    self.image_saved=False # ask next picture

                

    def image_callback(self, msg):
        if self.image_saved:
            return
            
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
        self.get_logger().info(f'    get image') 
        self.image=frame.copy()      
        self.image_saved = True
        self.track_atag()
        
    def move_cube(self):
        
        # self.publisher_.publish() with queue=20, thus, pub msg in one go.     

        st1=f'5 {self.f_atag_pm45} 0 0'
        self.publisher_.publish(String(data=st1)) 
        #time.sleep(1.0)

        st1="30 48 0 0" # camera-claw y-err is 48mm
        self.get_logger().info(f'{st1}')
        self.publisher_.publish(String(data=st1))      
        #time.sleep(1.5)

        st1=f"40 {self.f_atag_pm45} 0 0" # camera-claw x-err is 10mm and adjuest claw 
        self.get_logger().info(f'{st1}')
        self.publisher_.publish(String(data=st1))      
        #time.sleep(1.0)

        st1="12 0 0 0" # claw get cube
        self.get_logger().info(f'{st1}')
        self.publisher_.publish(String(data=st1))      
        #time.sleep(1.0)

        st1="20 0 0 10" # z +10 mm
        self.get_logger().info(f'{st1}')
        self.publisher_.publish(String(data=st1))      
        #time.sleep(1.5)

        st1="0 0 0 0" # servo 2,3,4 home
        self.get_logger().info(f'{st1}')
        self.publisher_.publish(String(data=st1))      
        #time.sleep(1.5)

        st1="5 0 0 0" # servo#5 home
        self.get_logger().info(f'{st1}')
        self.publisher_.publish(String(data=st1))      
        #time.sleep(1.0)

        st1="1 -90 0 0"
        self.get_logger().info(f'{st1}')
        self.publisher_.publish(String(data=st1))      
        #time.sleep(1.5)
        
        st1="11 0 0 0" # claw open
        self.get_logger().info(f'{st1}')
        self.publisher_.publish(String(data=st1))      
        #time.sleep(1.5)   

        st1="1 0 0 0" # servo#1 home
        self.get_logger().info(f'{st1}')
        self.publisher_.publish(String(data=st1))      
        #time.sleep(1.0)              

        self.get_logger().info(f'    quit')      
        self.flg1=False


def main(args=None):
    rclpy.init(args=args)
    node1 = PublisherNode()
    
    try:
        while rclpy.ok() and node1.flg1 :
            rclpy.spin_once(node1)
    except KeyboardInterrupt:
        pass
    finally: 
        if rclpy.ok():  
            node1.destroy_node()         
            rclpy.shutdown()            
    
if __name__ == '__main__':
    main()
