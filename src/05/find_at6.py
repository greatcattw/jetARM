#!/usr/bin/env python3
# encoding: utf-8

import cv2
import sys
import math
import numpy as np
from dt_apriltags import Detector

pixel_tolerence=20
pixel_move_long_or_short=40
move_long=10
move_short=5

def draw_tag(image, tag):
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
				
				

def main():
    # 讀取圖片
    image = cv2.imread("1.jpg")
    if image is None:
        print("無法讀取 1.jpg，請確認檔案是否存在。")
        return
    
    img_height, img_width = image.shape[:2]
    i_img_center_yy = int(img_height / 2)
    i_img_center_xx = int(img_width / 2 )
    print(f'img_center=({i_img_center_xx},{i_img_center_yy})')

    # 灰階轉換
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

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
        with open("st_argv_for_arm.txt","w",encoding="ascii") as file1:
            file1.write("1 0 0 0") # notic other application to quit
        
    else:
        # 只有一個 tag，直接取第一個
        tag = tags[0]

        at_center_x, at_center_y = tag.center.astype(int)
        print(f"偵測到 AprilTag ID: {tag.tag_id}")
        print(f"AT中心點座標: ({at_center_x}, {at_center_y})")
        img_between_x=at_center_x-i_img_center_xx
        img_between_y=at_center_y-i_img_center_yy
        st1=f"{img_between_x} {img_between_y}"
        print(f"img_between={st1}")
        
        
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

        print(f"arm_action = {arm_action_x} {arm_action_y}")
        st_argv_for_arm=f"20 {arm_action_x} {arm_action_y} 0"
        with open("st_argv_for_arm.txt","w",encoding="ascii") as file1:
            file1.write(st_argv_for_arm)

        if st_argv_for_arm=="20 0 0 0":
        
            corners = tag.corners
            p0 = corners[0]
            p1 = corners[1]

            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]

            angle_deg = math.degrees(math.atan2(dy, dx))
            angle_deg = (angle_deg + 180) % 360 - 180   # 正規化到 -180~180
            angle_pm45 = angle_deg - round(angle_deg / 90) * 90

            print(f"Tag angle = {angle_deg:.2f} deg")
            print(f"Angle in +/-45 = {angle_pm45:.2f} deg")
            with open("atag_pm45.txt","w",encoding="ascii") as file2:
                st1=f"5 {int(round(angle_pm45,0))} 0 0"
                print(st1)
                file2.write(st1)

        # img down is x+, right is y+
        # arm up is x+, left is y+
        # 標示 tag
        draw_tag(image, tag)

        height, width = image.shape[:2]
        center_x = width // 2
        center_y = height // 2
        # 畫垂直中心紅線
        cv2.line(
            image,
            (center_x, 0),
            (center_x, height - 1),
            (0, 0, 255),
            1
        )

        # 畫水平中心紅線
        cv2.line(
            image,
            (0, center_y),
            (width - 1, center_y),
            (0, 0, 255),
            1
        ) 
        cv2.rectangle(
            image,(center_x-pixel_tolerence, center_y-pixel_tolerence),
                  (center_x+pixel_tolerence, center_y+pixel_tolerence),(0, 255, 0),1)                 
        cv2.rectangle(
            image,(center_x-pixel_move_long_or_short, center_y-pixel_move_long_or_short),
                  (center_x+pixel_move_long_or_short, center_y+pixel_move_long_or_short),(0, 255, 255),1)                 
                  

    # 顯示結果
    cv2.imshow("AprilTag Detection", image)
    cv2.waitKey(1)  # 先讓視窗更新

    # 不帶命令列參數：等待按鍵
    if len(sys.argv) == 1:
        print('Press any key to quit.')
        cv2.waitKey(0)
    # 帶命令列參數：顯示 1 秒
    else:
        print('Show image for 1 second.')
        cv2.waitKey(1000)    
    #cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
