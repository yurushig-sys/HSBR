# -*- coding: utf-8 -*-
#
# High Speed Self Balance Robot (HSSBR)
# Field Hockey version
#    FH20210602
#    FH20210610 change procedure to find goals
#    FH20210618 find ball and kick
#    FH20210625 find ball and kick2
#    FH20210711 for the Field Hockey Ground 1
#    FH20210716 for the Field Hockey Ground 1, Overwrite PID parameters
#    FH20211212 for the Field Hockey simple version

version = "#4 FH20211212: for the Field Hockey simple version"

import time as tm
import cv2
import numpy as np
import serial
import threading

import calc as cc
import colorDetection as cD #user function
import controlArduino as cAd   #control ESP21 & Camera through serial connection
import control_box as cb
import globalVariables as gbv

print("start (version=%s)" %(version))

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
tm.sleep(4)
ser.reset_input_buffer()

# webカメラを扱うオブジェクトを取得
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
#gbv.frameRate = 30
#cap.set(cv2.CAP_PROP_FPS, gbv.frameRate)
#print("FPS = %d" %(cap.get(cv2.CAP_PROP_FPS)))
frame_rate = 60 # 90
WIDTH = 1280
HEIGHT = 720
cap.set(cv2.CAP_PROP_FPS, frame_rate)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
print("FPS = %d, WIDTH = %d, HEIGHT = %d"
      %(cap.get(cv2.CAP_PROP_FPS),
        cap.get(cv2.CAP_PROP_FRAME_WIDTH),
        cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

pBP = cb.pseudoBreakPoint()
csbr = cAd.controleSBR(ser, cap)

cbox = threading.Thread(target=cb.ControlBox, args=(ser, cap, ))

cbox.start() # start controll box
gbv.system_start = 0

w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

centerX = int(w/2)
centerY = int(h/2)

#pre_x = ""

t0 = tm.time()

csbr.initServoMotors()    #initialize all servo motors
#csbr.setPIDparms('P', 0.5, 0, 0.9, 20)  #overwrite Pos PID parameters

#global frame
ret,gbv.frame = cap.read()
if ret is False:
    print("cannot read image")
    exit()

pgc = cAd.place_to_global_coordinates(ser, cap, centerX, centerY)

ser.write(str.encode("za -4.5\n"))   #send command to the ESP32 to over write Angle offset
ser.write(str.encode("zYA 1.0 0 0.3 15\n"))
"""
#Call test function
cD.test1(cap, csbr, ser)
pBP.breakPoint("test", cap, csbr)
#"""

ballRadius = 60
color ="GREEN"
mode = 1  # mode = 1: simple one goal, mode = 2: two goals for each players
gbv.adjustMode = 1

while True:
    
    try:
        while gbv.system_start == 0 :   #wait unti system start
            pos = cD.find_color(cap, color)
            pass

        #wait until stand up
        csbr.checkStandUp()
        tm.sleep(1.0)
        csbr.presetSBR(0, 0, 0, 0)
        print("stand up")
        #pBP.breakPoint("stand up", cap, csbr)
        
        #Search GREEN Goal for distance adjustment
        dis = 0
        while dis == 0:
            dis, theta = pgc.find_target("GREEN", 0)  #find green Goal
            if dis > 0 :
                break
            print("Can not find Green Goal")
        
        if gbv.adjustMode == 1:
            cAd.distanceAdjust(cap, csbr, ser, centerX, centerY, ballRadius)
            gbv.adjustMode = 0
        #pBP.breakPoint("end distance adjustment", cap, csbr)
        #tm.sleep(0.1)
        
        rotSpeed = 30
        #r = 200 #150 #200 #300

        maxSpeedInput = 60 #65 #60 #100 #90
        initSpeedInput = 35 #50
        csbr.setMaxSpeed(maxSpeedInput, initSpeedInput)
        csbr.rotateRobot(0, 30)  #set Max Rotation speed to 30 (default = 60)
        
        while True:
            if mode == 2:  #two goals mode
                while True:
                    Ggx, Grx, Gry, Gtheta = pgc.find_goal_places()  #Grx, Gry, Gtheta are Zero
                    if Ggx > 0 and Ggx <=1600:
                        break
                    else :
                        print("Out of range: Ggx=%0.2f, Try again " %(Ggx))
        
                csbr.presetSBR(Grx, Gry, 0, Gtheta)
                #pBP.breakPoint("found goals", cap, csbr)

            #simple version: Find only Green Goal and Find ball, then Hit the ball to the Green Goal
            elif mode == 1:  #simple one goal mode
                while True:
                    Ggx, Gtheta = pgc.find_target("GREEN", 0)  #find green Goal
                    Grx, Gry, Gtheta = 0, 0, 0
                    if Ggx <= 1700.0 and Ggx >= 300.0:
                        #pBP.breakPoint("found goals: Ggx=%0.2f" %(Ggx), cap, csbr)
                        break
                    else :
                        print("Out of range: Ggx=%0.2f, Try again " %(Ggx))
                        #pBP.breakPoint("could not find any goal", cap, csbr)
            
                print("Goal distance=%0.2f Gtheta=%0.2f" %(Ggx, Gtheta))
                csbr.presetSBR(Grx, Gry, 0, Gtheta)
                #pBP.breakPoint("found green goal", cap, csbr)
            
            #find ball and calculate hitting path
            while True:
                csbr.setStickAngle(90)
                while True:
                    L1, Th1 = pgc.find_target("RED", ballRadius)
                    if L1 > 0:
                        break
                    else:
                        #pBP.breakPoint("could not find ball", cap, csbr)
                        pass
                    
                print("Ball distance=%0.2f Theta=%0.2f Rx=%0.2f Ry=%0.2f" %(L1, Th1, gbv.Wx, gbv.Wy))
                #Th2, L1_, r, Th5 = pgc.find_ball(Ggx)
                #pBP.breakPoint("found red ball", cap, csbr)
                csbr.setStickAngle(50)
                fTurn, run, sTurn, r = cc.calcOrbit(gbv.Wx, gbv.Wy, Ggx, L1, np.radians(Th1))
                #make continuous comman
                #contCmd = ("%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f"
                #     %(0, np.degrees(fTurn), 10000, run, r, np.degrees(sTurn), 10000, 200))
                contCmd = ("%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f"
                     %(0, np.degrees(fTurn), 10000, run, r, np.degrees(sTurn)))
                print(contCmd)
                csbr.setContCmd(contCmd)
                if np.degrees(sTurn) < 45.0: 
                    pgc.checkContinuousFlag(2, "RED")
                else:
                    pgc.checkContinuousFlag(4, "RED")
                csbr.setStickAngle(150)
                pgc.checkContinuousFlag(6, "RED")
                tm.sleep(0.5)
                csbr.setStickAngle(90)
                csbr.setTargetLp(-150)  #set back 300mm
                pBP.breakPoint("Hit Ball", cap, csbr)
                #tm.sleep(3.0)
                break
        
    except Exception as e:
        print("exception", e)
        if str(e) == "fall down" :
            #positionInz_flag = 0
            #gbv.system_start = 0
            csbr.initServoMotors()
            if gbv.system_start == 1:
                csbr.turnDown()
                csbr.standUp()
        #elif gbv.system_start == 1 and gbv.start_flag == 0:
        #    csbr.standUp()
            
        print("end Exception")
        print(type(e))


    
