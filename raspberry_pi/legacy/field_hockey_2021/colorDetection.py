# -*- coding: utf-8 -*-

#  color detect algorithm by
#    https://gist.github.com/TonyMooori/4cc29c94f7bdbade6ff6102fef45232e

import cv2
import numpy as np
import globalVariables as gbv
import time as tm

#Color of the Target Ball, defined by hsv color space
#note, in opencv, H=0~180(not 0~360), S&V=0~255(not 0~100%)
#


def find_specific_color(cap, AREA_RATIO_THRESHOLD,LOW_COLOR,HIGH_COLOR):
    
    ret,gbv.tempFrame = cap.read()
    # 高さ，幅，チャンネル数
    h,w,c = gbv.tempFrame.shape

    # hsv色空間に変換
    hsv = cv2.cvtColor(gbv.tempFrame,cv2.COLOR_BGR2HSV)
    
    # 色を抽出する
    ex_img = cv2.inRange(hsv,LOW_COLOR,HIGH_COLOR)
    #cv2.imshow("exx_img", ex_img)
    # 輪郭抽出
    #_,contours,hierarchy = cv2.findContours(ex_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours,hierarchy = cv2.findContours(ex_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    # 面積を計算
    areas = np.array(list(map(cv2.contourArea,contours)))

    if len(areas) == 0 or np.max(areas) / (h*w) < AREA_RATIO_THRESHOLD:
        # 見つからなかったらNoneを返す
        #print("the area is too small", len(areas))
        cv2.imshow('frame',gbv.tempFrame)
        cv2.moveWindow('frame',640, 320) #1280, 500) 
        key = cv2.waitKey(1) & 0xFF
        return None
    else:
        # 面積が最大の塊の重心を計算し返す
        max_idx = np.argmax(areas)
        max_area = areas[max_idx]
        ((ECx, ECy), ECr) = cv2.minEnclosingCircle(contours[max_idx])  # add this line
        result = cv2.moments(contours[max_idx])
        x = int(result["m10"]/result["m00"])
        y = int(result["m01"]/result["m00"])
        #print(x, y, max_area, ECx, ECy, ECr)
        gbv.frame = cv2.circle(gbv.tempFrame,(int(ECx), int(ECy)),50,(255, 255, 255), 10)
        cv2.imshow('frame',gbv.tempFrame)
        #cv2.moveWindow('frame', 1280, 500) #1280, 550)
        key = cv2.waitKey(1) & 0xFF
        return (x,y, max_area, ECx, ECy, ECr)

#
#find specific color target
#
Blue_LOW = np.array([100, 100, 50]) #([95,50,50]) #([100, 50, 50]) #([100, 170, 100])    #near dark blue
Blue_HIGH = np.array([110,255,255]) #([110, 255, 255])  #near bright blue
Blue_AREA_RATIO_THRESHOLD = 0.0001 #0.00005

Green_LOW = np.array([60,50,50]) #([70, 50, 50])    #near dark blue
Green_HIGH = np.array([90, 255, 255])  #near bright blue
#Green_AREA_RATIO_THRESHOLD = 0.0001 #0.001  for #1: 0.6m @0.01, 1.5m @0.001
Green_AREA_RATIO_THRESHOLD = 0.0001 #for #2  0.85m @ 0.001, 1.35m @ 0.0001, 1.50m @ 0.00005

Orange_LOW = np.array([5, 150, 150])    #near dark orange
#Orange_HIGH = np.array([10, 255, 255])  #near bright orange
Orange_HIGH = np.array([15, 255, 255])  #near bright orange
Orange_AREA_RATIO_THRESHOLD = 0.01

#Red_LOW = np.array([160, 100, 100])
Red_LOW = np.array([165, 100, 100]) #([170, 50, 50])
#Red_HIGH = np.array([180, 255, 255])
Red_HIGH = np.array([180, 255, 255])
#Red_AREA_RATIO_THRESHOLD = 0.0001 #for #1  0.6m @0.01 , 1.3m @0.001  1.6m @0.0005
Red_AREA_RATIO_THRESHOLD = 0.0001 #for #2, 0.85m @ 0.001, 1.35m @ 0.0001  1.50m @ 0.00005

def find_color(cap, color):
    
    if color == "BLUE":
        pos = find_specific_color(cap, Blue_AREA_RATIO_THRESHOLD, Blue_LOW, Blue_HIGH)
    elif color == "GREEN":
        pos = find_specific_color(cap, Green_AREA_RATIO_THRESHOLD, Green_LOW, Green_HIGH)
    elif color == "ORANGE":
        pos = find_specific_color(cap, Orange_AREA_RATIO_THRESHOLD, Orange_LOW, Orange_HIGH)
    elif color == "RED":
        pos = find_specific_color(cap, Red_AREA_RATIO_THRESHOLD, Red_LOW, Red_HIGH)
    else :
        pos = None
    
    return pos

############################################################################
## test programs
############################################################################import math
import math
centerX, centerY = 640, 360
H_viewAngle, V_viewAngle = 48/2, 28/2
robotHight = 170
adjustment = 0.0  #bigger is longer
Vth = 105.0 #106.0

def calcD(color, pos):
    x, y, maxarea, ECx, ECy, ECr = (pos)
    cx = centerX - x
    hth = H_viewAngle * (cx/centerX) #[degree]
                
    cy = y - centerY
    vth = V_viewAngle * (cy/centerY)
    vtemp = (180 - (Vth + vth ) + adjustment - gbv.Angle) 
    distance = robotHight * math.tan(np.radians(vtemp))
    distance += 40.0
    print("[%s] x=%0.2f y=%0.2f distance=%0.2f[mm] vtemp=%0.2f[deg] fAngle=%0.2f" %(color, x, y, distance, vtemp, gbv.Angle))
    return distance, hth
    
def test1(cap, csbr, ser):
    
    while True:
        
        csbr.getData()
        posRed = find_color(cap, "RED")
        posGr = find_color(cap, "GREEN")
        cv2.line(gbv.tempFrame, (0, 360), (1280, 360), (255, 255, 255), thickness=1, lineType=cv2.LINE_4)
        cv2.line(gbv.tempFrame, (640, 0), (640, 720), (255, 255, 255), thickness=1, lineType=cv2.LINE_4)
        
        if posRed is not None:
            x, y, maxarea, ECx, ECy, ECr = (posRed)
            gbv.tempFrame = cv2.circle(gbv.tempFrame,(int(ECx), int(ECy)),int(ECr*1.2),(0, 0, 255), 5)
            cv2.arrowedLine(gbv.tempFrame,(640,720),(int(ECx),int(ECy)),(0,0,255),thickness=2,tipLength=0.08)
            distance, hthRed = calcD("RED", posRed)
            dis=("%0.1fcm" %(distance/10))
            cv2.putText(gbv.tempFrame, dis ,(0,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255),2)
            th=("%0.1fdegree" %(hthRed))
            cv2.putText(gbv.tempFrame, th ,(150,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255),2)
            print("posRed=", posRed)
        
        if posGr is not None:
            x, y, maxarea, ECx, ECy, ECr = (posGr)
            gbv.tempFrame = cv2.circle(gbv.tempFrame,(int(ECx), int(ECy)),int(ECr*1.2),(0, 255, 0), 5)
            cv2.arrowedLine(gbv.tempFrame,(640,720),(int(ECx),int(ECy)),(0,255,0),thickness=2,tipLength=0.08)
            distance, hthGr = calcD("GREEN", posGr)
            dis=("%0.1fcm" %(distance/10))
            cv2.putText(gbv.tempFrame, dis ,(0,100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0),2)
            th=("%0.1fdegree" %(hthGr))
            cv2.putText(gbv.tempFrame, th ,(150,100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0),2)
            #print("posGr=", posGr)
        
        gbv.frame = gbv.tempFrame
        cv2.imshow('frame',gbv.frame)

        #key = cv2.waitKey(1) & 0xFF
        if cv2.waitKey(100) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
  
        if posRed is not None and posGr is not None:
            th = (hthRed + hthGr)/2
        elif posRed is not None:
            th = hthRed
        elif posGr is not None:
            th = hthGr
        else:
            th = 0
        
        speed = 30
        """
        csbr.rotateRobot(th*0.5, speed)
        """