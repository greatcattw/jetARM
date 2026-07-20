import cv2
import math

from params_bottle_move import (
    pixel_tolerence,
    pixels_per_30mm,
    pixel_move_long_or_short,
    move_long,
    move_short,
)

class Gcat_utils:
    def __init__(self, logger):
        self.logger = logger
        #self.servo_pub = servo_pub


    def draw_tag(self,image, tag,pixel_tolerence,pixel_move_long_or_short):
        """
                 參考附件方式標示 AprilTag：
        - 紅色畫四個角點
        - 綠色畫中心點
        - 顯示 tag_id 與中心座標
        """
        corners = tag.corners.astype(int)
        center = tuple(tag.center.astype(int))

        # 畫四個角點連線
        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(image, pt1, pt2, (0, 0, 255), 2)

        # 畫角點
        for corner in corners:
            cv2.circle(image, tuple(corner), 5, (0, 0, 255), -1)

        # 畫中心點
        cv2.circle(image, center, 5, (0, 255, 0), -1)

        # 顯示 tag id 與中心座標
        #text = f"ID:{tag.tag_id} Center:{center}"
        #cv2.putText(image, text, (center[0] + 10, center[1] - 10),
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        img_height, img_width = image.shape[:2]
        i_img_center_yy = int(img_height / 2)
        i_img_center_xx = int(img_width / 2 )
    				
        # 畫垂直中心紅線
        cv2.line(
            image,
            (i_img_center_xx, 0),
            (i_img_center_xx, img_height - 1),
            (0, 0, 255),
            1
        )
        # 畫水平中心紅線
        cv2.line(
            image,
            (0, i_img_center_yy),
            (img_width - 1, i_img_center_yy),
            (0, 0, 255),
            1
        ) 

        cv2.rectangle(
            image,(i_img_center_xx-pixel_tolerence, i_img_center_yy-pixel_tolerence),
                  (i_img_center_xx+pixel_tolerence, i_img_center_yy+pixel_tolerence),(0, 255, 0),1)                 
        cv2.rectangle(
            image,(i_img_center_xx-pixel_move_long_or_short, i_img_center_yy-pixel_move_long_or_short),
                  (i_img_center_xx+pixel_move_long_or_short, i_img_center_yy+pixel_move_long_or_short),(0, 255, 255),1) 
                  
                  
    def track1(self,img_between_x,img_between_y):
        # img +x is to right, mapping arm  -y to right
        if abs(img_between_x) < pixel_tolerence :
            arm_action_y="0"
        elif img_between_x > 0 :
            if img_between_x > pixel_move_long_or_short :           
                arm_action_y=f"{move_long *-1}" 
            else:
                arm_action_y=f"{move_short *-1}"
        else:
            if img_between_x < -pixel_move_long_or_short :           
                arm_action_y=f"{move_long}"
            else:
                arm_action_y=f"{move_short}"

        # img +y is to down, mapping arm  -x to down    
        if abs(img_between_y) < pixel_tolerence :
            arm_action_x="0"
        elif img_between_y > 0 :
            if img_between_y > pixel_move_long_or_short :           
                arm_action_x=f"{move_long *-1}" 
            else:
                arm_action_x=f"{move_short *-1}"            
        else:
            if img_between_y < -pixel_move_long_or_short :           
                arm_action_x=f"{move_long}" 
            else:
                arm_action_x=f"{move_short}"            
        st1=f'{arm_action_x} {arm_action_y}'
        self.logger.info(f"arm_action = {st1}")
        return(st1)

        
    def cal_atag_pm45(self,corners):
            p0 = corners[0]
            p1 = corners[1]

            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]

            angle_deg = math.degrees(math.atan2(dy, dx))
            angle_deg = (angle_deg + 180) % 360 - 180   # 正規化到 -180~180
            angle_pm45 = angle_deg - round(angle_deg / 90) * 90

            self.logger.info(f"Tag angle = {angle_deg:.2f} deg")
            self.logger.info(f"Angle in +/-45 = {angle_pm45:.2f} deg")
            #f_pm45=round(angle_pm45,0)
            f_pm45=angle_pm45
            return(f_pm45)
            '''
            with open("atag_pm45.txt","w",encoding="ascii") as file2:
                st1=f"5 {int(round(angle_pm45,0))} 0 0"
                self.logger.info(st1)
                file2.write(st1)
            '''    
    def plan_arm_action(self,at_center_x,at_center_y,i_img_center_xx,i_img_center_yy,tag1):
        img_between_x=at_center_x-i_img_center_xx
        img_between_y=at_center_y-i_img_center_yy
        st1=f"{img_between_x} {img_between_y}"
        self.logger.info(f"img_between={st1}")

        if (abs(img_between_x)>pixels_per_30mm) or (abs(img_between_y)>pixels_per_30mm) :
            move_far_x=round((abs(img_between_x) / pixels_per_30mm)*30,0)
            if img_between_x > 0 :
                arm_action_y=f"{move_far_x *-1}" 
            else:
                arm_action_y=f"{move_far_x}"

            move_far_y=round((abs(img_between_y) / pixels_per_30mm)*30,0)
            if img_between_y > 0 :
                arm_action_x=f"{move_far_y *-1}" 
            else:
                arm_action_x=f"{move_far_y}"
            st1=f'50 {arm_action_x} {arm_action_y} 0'
            self.logger.info(f"arm_action = {st1}")                
        else:
            st2=self.track1(img_between_x,img_between_y)
            self.logger.info(f"111111111111111111111111111111111111") 
            st1=f'50 {st2} 0'
        return(st1)
        '''    
        st_argv_for_arm=f"20 {st1} 0"
        with open("st_argv_for_arm.txt","w",encoding="ascii") as file1:
            file1.write(st_argv_for_arm)

        if st_argv_for_arm=="20 0 0 0":
        
            cal_atag_pm45(tag1.corners)                  
        '''          
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  
                  
