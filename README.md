# jetARM
hiwonder jetARM study  
# 警告:程式可能導致手臂撞機的風險, 無法承擔風險的人請跳離
# 警告:程式可能導致手臂撞機的風險,使用時,手要放在電源開關上,異常時即時斷電  
# In progress
參考一招由天而降的掌法, 悟出由天而降的爪法, 避開了視角計算難題 <br>
## 01 check shift of camera and claw, 10mm 47mm
[ex01](src/01/ex01.md)<br>
<br>![pic](pic/01/01_func.png)<br>
<br>
## 02 check shift of camera and claw, 20mm
目前沒用到,  
[ex02](src/02/02.md)<br>
<br>![pic](pic/02/02.png)<br>
<br>
## 03 絕對定位 數學基礎
src/03/簡單手臂的逆運動學2.pdf<br>
顧好腕的位置<br>
<br>![pic](pic/03/d2.png)<br>
<br>![pic](pic/03/a8.PNG)<br>
<br>
## 04 用命令行參數,操作手臂移動
校正姿勢<br>
腕的絕對定位<br>
腕的相對定位<br>
腕徑向伸長<br>
腕逆時針旋轉10mm<br>
舵機1,2,3,4,5,10操作<br>
爪子動作<br>
[ex04](src/04/04.md)<br>
## 05 抓取有April tag的木塊  
![pic](pic/05/1.png)<br>
分為2個步驟,調好參數再用AI融成一支程式<br>
1 鏡頭中心對準木塊中心<br>
2 爪子抓木塊<br>
[ex05](src/05/05.md)<br>

## 06 AI整理程式pub6.py - > pub6a.py
1. 用 math.atan2() 取代 math.atan(y / x)，避免除以 0<br>
2. pub6.py拆成三個.py <br>
├── sub6a.py <br>
├── servo_utils.py <br>
└── params.py <br>
## 07 影像中心與爪子的偏差
影像中心的xy移動<br>
![pic](pic/07/a2.PNG)<br>
