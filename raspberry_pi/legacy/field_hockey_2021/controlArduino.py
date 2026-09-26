# -*- coding: utf-8 -*-
#
# function control arduino
#

import time as tm
import colorDetection as cD
import control_box as cb
import globalVariables as gbv
import numpy as np

V_viewAngle = 28.0 / 2  #degree,  by simple measurement
H_viewAngle = 48.0 / 2  #degree,  by measurement
robotHight = 170 #[mm]  for HSSBR-II
adjustment = 0  #bigger is longer
Vth = 104 #106.0
BLUE = 1
GREEN = 2
    
class place_to_global_coordinates():
    
    def __init__(self, ser, cap, centerX, centerY):
        
        self.ser = ser
        self.cap = cap
        self.centerX = centerX
        self.centerY = centerY
        
        #self.cms = controleCameraServo(self.ser, self.cap)
        self.csbr = controleSBR(self.ser, self.cap)
        
        self.setback = 300  #setback 300mm
        
        self.pBP = cb.pseudoBreakPoint()
    """
    def find_goal_places(self):
        N = 10
        bDis, gDis = 0, 0

        while True:       #search BLUE goal
            bDis, bTh = self.find_target("BLUE", 0)
            if bDis != 0:
                break
            else:
                gDis, gTh = self.find_target("GREEN", 0)
                if gDis == 0:
                    raise Exception ("cannot find any goal")
                #could not find blue but green, assume BLUE goal is too near
                self.csbr.setTargetLp(100)  #move 300mm fotward GREEN goal
                self.csbr.checkMoveFlag()
                tm.sleep(1.0)
       
        #found BLUE goal
        #print("found BLUE goal: Dis=%0.1f" %(bDis))
        #self.pBP.breakPoint("found BLUE goals", self.cap, self.csbr)

        #print("BLUE goak (Average): Dis=%0.1f Th=%0.1f" %(bDis, bTh))
        #self.pBP.breakPoint("found BLUE goals", self.cap, self.csbr)
        wx, wy, lp, theta = 0, 0, 0, 0
        self.csbr.presetSBR(wx, wy, lp, theta)    #set new local coordinate
        
        #search GREEN goal
        gDis, gTh = self.find_target("GREEN", 0)   #[mm], [deg]
        if gDis == 0:
            #could not find green, assume green goal is too near
            self.csbr.rotateAbsoluteAngle(0, 30)
            self.csbr.setTargetLp(100)  #move 300mm fotward BLUE goal
            self.csbr.checkMoveFlag()
            tm.sleep(1.0)
            
            gDis, gTh = self.find_target("GREEN", 0)  #search GREEN goal again
            if gDis == 0:
                raise Exception ("cannot find any goal")

        print("found goals: bDis=%0.2f  gDis=%0.2f gTh=%0.2f" %(bDis, gDis, gTh))

        # convert local coordinate to global coordinata
        #self.csbr.getData()   #get current coordinata and direction   
        Lrx, Lry = gbv.Wx, gbv.Wy               #robot x, y
        #Ltheta = gbv.Theta % 360                #robot direction
        Lbx, Lby = bDis, 0
        Lgx, Lgy = gDis*np.cos(np.radians(gTh))+Lrx, gDis*np.sin(np.radians(gTh))+Lry
        print("Local R(%0.2f, %0.2f) gTh=%0.2f B(%0.2f, %0.2f) G(%0.2f, %0.2f)" %(Lrx,Lry,gTh,Lbx,Lby,Lgx,Lgy))
        
       #shift Blue goal (Lbx, Lby) to origin of coordinate (0, 0)
        Lgx = Lgx - Lbx
        Lrx = Lrx - Lbx
        
        lgb = np.sqrt(Lgx**(2) + Lgy**(2))
        
        thGBR = np.pi - np.arcsin(Lgy / lgb)  #[rad]      
        print("lgb=%0.2f thGBR=%0.2f" %(lgb, np.digrees(thGBR)))
        
        #rotate Local coordinate - thGBR at BLUE goal as an origin
        Ggx = Lgx * np.cos(-thGBR) - Lgy * np.sin(-thGBR)
        Ggy = Lgx * np.sin(-thGBR) + Lgy * np.cos(-thGBR)
        Grx = Lrx * np.cos(-thGBR) - Lry * np.sin(-thGBR)
        Gry = Lrx * np.sin(-thGBR) + Lry * np.cos(-thGBR)
        #Gtheta = (Ltheta + (np.digrees(thbr))) % 360
        Gtheta = gTh - np.degrees(thGBR).pi
        
        print("Gg(%0.2f, %0.2f) Gr(%0.2f, %0.2f) Gtheta=%0.2f" %(Ggx, Ggy, Grx, Gry, Gtheta)) 
        
        return Ggx, Grx, Gry, Gtheta
    
    #find_ball
    def find_ball(self, gGx):   #gGX is green goal x
        r = 100 #[mm]
        ballRadius = 50 #30
        L1 = 0    
        while True:       #search RED ball goal
            L1, Th1 = self.find_target("RED", ballRadius)  #L1:[mm], Th1:[deg]
            Th1 = np.radians(Th1)  #[rad]
            if Th1 > np.pi :
                Th1 = Th1 - np.pi*2  #Th1 must be withi 0 ~ 180 or 0 ~ -180
            if L1 != 0:
                break
            #else:
            #    tx, ty = 0, gGx /2            
            #    self.csbr.setTargetPosition(tx, ty)  #move to the center of the coat
       
        print("found RED ball: L1=%0.2f Th1=%0.2f" %(L1, np.digrees(Th1)))
        Rx, Ry = gbv.Wx, gbv.Wy
        Bx = L1 * np.cos(Th1) + Rx
        By = L1 * np.sin(Th1) + Ry
        Th2 = np.arcsin(r / L1)  #[rad]
        Th3 = np.arctan((0 - By)/(gGx - Bx))  #Th3 = 0 to -pi/2, 0 to +pi/2
        print("Rx=%0.2f Ry=%0.2f Bx=%0.2f By=%0.2f Th2=%0.2f Th3=%0.2f"
              %(Rx, Ry, Bx, By, np.digrees(Th2), np.digrees(Th3)))
        #equation of line (Bx, By) to (gGx, 0) :  y = cx + d
        c = -By/(gGx - Bx)
        d = -c * Bx + By   #By*Bx/(gGx - Bx) + By
        #Hitting point
        Hx = r * np.cos(np.pi/2 + Th3) + Bx
        Hy = r * np.sin(np.pi/2 + Th3) + By
        #y = cx + f
        c = -By / (gGx - Bx)
        f = - c * Hx + Hy
        #judge the position of the ball against the robot
        y = c * Bx + f
        if y < Ry + 5.0 and y > Ry - 5.0:
            position = 0 #neutral,  don't need wraparound
        elif y >= Ry + 5.0:
            position = 1 #upper
        else:
            position = -1 #lower
        
        Th4 = np.pi - abs(Th1 + Th2) - abs(Th3)
        Th5 = np.pi - Th4
        
        if position == 1:
            L1_ = L1 * np.cos(Th2)
            print("position=%d L1_=%0.2f r=%0.2f Th5=%0.2f" %(position, L1_, r, np.degrees(Th5)))
            return Th2, L1_, r, -Th5
        if position == -1:
            #rotate the robot-ball line to -Th2
            Bx_ = np.cos(-Th2) * (Bx - Rx) - np.sin(-Th2) * (By - Ry) +Rx
            By_ = np.sin(-Th2) * (Bx - Rx) + np.cos(-Th2) * (By - Ry) +Ry
            print("Bx=%0.1f By=%0.1f Bx_=%0.1f By_=%0.1f Th2=%0.1f" %(Bx, By, Bx_, By_, np.degrees(Th2)))
            #y = ax + b which connect (Rx, Ry) to (Bx_ , By_)
            a = (By_ - Ry) / (Bx_ - Rx)
            b = -(By_ - Ry) / (Bx_ - Rx) * Rx + Ry
            print("a=%0.1f b=%0.1f c=%0.1f f=%0.1f" %(a, b, c, f))
            #cross point of y = ax + b and y = cx + f
            Cx = (f - b) / (a - c)
            Cy = (a*f - c*b) / (a - c)
            print("Hx=%0.1f Hy=%0.1f Cx=%0.1f Cy=%0.1f" %(Hx, Hy, Cx, Cy))
            Lch = np.sqrt((Hx - Cx)**(2) + (Hy - Cy)**(2))
            Lt = np.sqrt((Cx - Rx)**(2) + (Cy - Ry)**(2)) - Lch
            r2 = Lch / np.tan(Th5/2)
            print("position=%d Lch=%0.2f Lt=%0.2f r2=%0.2f Th5=%0.2f" %(position, Lch, Lt, r2, np.degrees(Th5)))
            return -Th2, Lt, r2, Th5
        if position == 0:
            L1_ = L1 * np.cos(Th2)
            print("position=%d Lt=%0.2f r2=%0.2f Th5=%0.2f" %(position, L1_, 0, 0))
            return Th2, L1_, 0, 0
    """

    def find_target(self, color, targetHight):
        #print("search target ", color)
        self.csbr.rotateRobot(360, 30)
        
        while gbv.rotate_flag == 1:
            pos = cD.find_color(self.cap, color)
            if pos is not None:
                self.csbr.haltSBR()
                self.csbr.checkRotateFlag()
                dis, th = self.calcDistance(color, pos, targetHight)
                return dis, th
            
            self.csbr.getStatus()
        
        print("cannot find target ", color)
        return 0, 0
    
    def calcDistance(self, color, pos, targetHight):
        
        while True:   #adjust Horizontal Center to the target
            x, y, maxarea, ECx, ECy, ECr = (pos)
            cx = self.centerX - x
            cy = self.centerY - y
            hth = H_viewAngle * (cx/self.centerX) #[degree]
            vth = V_viewAngle * (cy/self.centerY)
            #print("x=%d y=%d hth=%0.2f vth=%0.2f Vth_=%0.2f" %(x, y, hth, vth, Vth_))
            if abs(hth) <= 2.0: #and abs(vth) <= 0.1:
                break
            if abs(hth) > 2.0:
                self.csbr.rotateRobot(hth/2, 40)
                self.csbr.checkRotateFlag()
            #if abs(vth) > 0.1:
            #    Vth_ -= vth/2
            #    self.cms.controlCamera(Hth, Vth_ , self.ser)
            #    tm.sleep(0.2)
                
            pos = cD.find_color(self.cap, color)
            if pos == None:
                print("lost [%s] in calcDistance" %(color))
                #self.pBP.breakPoint("lost [%s]" %(color), self.cap, self.csbr)
                #self.cms.controlCamera(Hth, Vth, self.ser)
                #raise Exception ("lost goal while serarching goal %s" %(color))
                return 0, 0


        #gbv.Theta = 0
        #self.csbr.getData()  #get gbv.Wx, gbv.Wy, gbv.Theta
        N = 4 #10
        sumD, sumA = 0, 0
        n = N
        while n > 0:
            self.csbr.getData() 
            pos = cD.find_color(self.cap, color)
            while gbv.que_flag == 1:
                pass
            if pos == None:
                print("lost [%s] in calcDistance Adjestment" %(color))
                #self.pBP.breakPoint("lost [%s] in calcDistance" %(color), self.cap, self.csbr)
                #self.cms.controlCamera(Hth, Vth, self.ser)
                return 0, 0
            x, y, maxarea, ECx, ECy, ECr = (pos)
            cx = self.centerX - x
            hth = H_viewAngle * (cx/self.centerX) #[degree]
            cy = self.centerY - y
            vth = V_viewAngle * (cy/self.centerY)
            hight = robotHight - targetHight
            dx = hight * np.tan(np.radians((180 - (Vth - vth) + gbv.adjustment - gbv.Angle)))
            dx += 40
            distance = dx / np.cos(np.radians(hth))
            print("[%s] x=%0.2f y=%0.2f distance=%0.2f[mm] vth=%0.2f[deg] fAngle=%0.2f"
                  %(color, x, y, distance, vth, gbv.Angle))
    
            #print("[%s] D=%0.2f Th=%0.2f  Angle=%0.2f vth=%0.2f cy=%d hth=%0.2f dx=%0.2f"
            #      %(color, distance, gbv.Theta, gbv.Angle, Vth - vth, cy, hth, dx))
            #self.pBP.breakPoint("wait [%s]" %(color), self.cap, self.csbr)
            #distance = dx/np.cos(np.radians(hth))
            if distance > 2500 :
                print("distance is too long, more than 2500[mm]", distance)
                #self.cms.controlCamera(Hth, Vth, self.ser)
                return 0, 0
            sumD += distance
            sumA += gbv.Angle
            n -= 1
        
        distance = sumD / N
        angle = sumA / N
        #theta = np.arctan((distance - 40) / robotHight + np.radians(angle))
        #distance2 = robotHight * np.tan(theta) + 40.
        theta = np.arctan((distance - 40) / hight + np.radians(angle))
        distance2 = hight * np.tan(theta) + 40.
        #print("distance=%0.2f angle=%0.2f: theta=%0.2f distance2=%0.2f"
        #      %(distance, angle, theta, distance2))
        print("distance2=%0.2f[mm] Theta=%0.2f hth=%0.2f" %(distance2, gbv.Theta, hth))
        #self.pBP.breakPoint("wait [%s]" %(color), self.cap, self.csbr)
        #self.cms.controlCamera(Hth, Vth, self.ser)
        return distance2, (gbv.Theta + hth) % 360    
        
    def moveToward(self, coordinates):  #move 300mm toward specified goal
        gx, gy, distance = (coordinates)
        th = np.degrees(np.arctan((gy - gbv.Wy) / (gx - gbv.Wx)))
        self.csbr.rotateAbsoluteAngle(th, 30)  #rotate to specified goal
        self.csbr.setTargetLp(300)             #move 300mm fotward specified goal
     
    def checkContinuousFlag(self, paramNo, color):
        
        while True:
            pos = cD.find_color(self.cap, color)
            self.csbr.getStatus()
            if gbv.continuous_flag == 0:
                return gbv.paramIndex
            elif gbv.paramIndex > paramNo :
                return gbv.paramIndex
                
            

#
# function Control Self Balancing Robot(SBR)
#
class controleSBR():
    def __init__(self, ser, cap):
        self.ser = ser
        self.cap = cap
        #self.cms = controleCameraServo(self.ser, self.cap)
        #self.pBP = cb.pseudoBreakPoint(self.ser, self.cap)
        
    def standUp(self):
        self.ser.write(str.encode("zu1\n"))
        
    def turnDown(self):
        #self.cms.initialPosition()
        self.ser.write(str.encode("zu0\n"))
        
    def logOn(self):
        self.ser.write(str.encode("ze1\n"))

    def logOff(self):
        self.ser.write(str.encode("ze0\n"))
        print("log off")
    
    def initServoMotors(self):
        self.ser.write(str.encode("zsi\n"))
        
    def setCurrentPosition(self, wx, wy, wth):
        self.ser.write(str.encode("zr %0.2f %0.2f %0.2f\n" %(wx, wy, wth)))
    
    def setTargetPosition(self, tx, ty):
        gbv.rotate_flag = 1
        gbv.move_flag = 1
        self.ser.write(str.encode("zt %0.2f %0.2f\n" %(tx, ty)))
    
    def setTargetPosition2(self, t1x, t1y, t2x, t2y):
        gbv.rotate_flag = 1
        gbv.move_flag = 1
        self.ser.write(str.encode("zt %0.2f %0.2f %0.2f %0.2f\n" %(t1x, t1y, t2x, t2y)))
        
    def rotateRobot(self, th, speed):
        gbv.rotate_flag = 1     
        self.ser.write(str.encode("zo %0.2f %0.2f\n" %(th, speed)))
        
    def rotateAbsoluteAngle(self, th, speed):
        gbv.rotate_flag = 1     
        self.ser.write(str.encode("zO %0.2f %0.2f\n" %(th, speed)))
        
    def setContCmd(self, contCmd):   #set continuous command
        gbv.move_flag = 1
        gbv.rotate_flag = 1
        self.ser.write(str.encode("zc %s\n" %(contCmd)))
        
    def setTargetLp(self,tlp):
        gbv.move_flag = 1
        self.ser.write(str.encode("zp %0.2f\n" %(tlp)))
        
    def setStickAngle(self, left_stick): #right_stick, left_stick):
        self.ser.write(str.encode("zss %4d\n" %(left_stick)))
        
    def resetSBR(self):
        self.ser.write(str.encode("zr\n"))
        
    def presetSBR(self, wx, wy, lp, theta):
        self.ser.write(str.encode("zr %0.2f %0.2f %0.2f %0.2f\n" %(wx, wy, lp, theta)))
        
    def haltSBR(self):
        self.ser.write(str.encode("zh\n"))
        
    def setMaxSpeed(self, maxSpeedInput, initSpeedInput):
        self.ser.write(str.encode("zx %.0f %.0f\n" %(maxSpeedInput, initSpeedInput)))

    #def setStopRotation(self):
        #self.ser.write(str.encode("STOPR\n"))
 
    def setPIDparms(self, types, P, I, D, N):
        self.ser.write(str.encode("zY%s %0.1f %0.1f %0.1f %0.1f\n" %(types, P, I, D, N)))
 
    def getData(self):
        while gbv.que_flag == 1:
            pass
        gbv.que_flag = 1 
        self.ser.write(str.encode("zd1\n"))
        self.waitQue("zd1\n")
 
    def getStatus(self):
        while gbv.que_flag == 1:
            pass
        old_start_flag = gbv.start_flag
        gbv.que_flag = 1        
        self.ser.write(str.encode("zd0\n"))
        self.waitQue("zd0\n")
        if gbv.start_flag == 0 and old_start_flag == 1:
            raise Exception("fall down")

    def waitQue(self, cmd):
        retry_counter = 0
        while True:
            #t = tm.monotonic()
            t=tm.time()
            while gbv.que_flag == 1:
                ret,gbv.frame = self.cap.read()
                if tm.time() - t > 5.0 :
                    print(tm.time()-t)
                    print(retry_counter, t)
                    #self.pBP.breakPoint("retry", self.cap)
                    if retry_counter < 2:
                        retry_counter += 1
                        #self.ser.write(str.encode("%s\n") %(cmd))
                        self.ser.write(str.encode(cmd))
                        print(retry_counter)
                        #break
                    else:
                        raise Exception("getData or getStatus time over")
                
            break
        

    def checkRotateFlag(self):
        t = tm.monotonic()
        while gbv.rotate_flag == 1:
            ret,gbv.frame = self.cap.read()
            print("rotate_flag =%d  GTheta=%0.2f" %(gbv.rotate_flag, gbv.Theta))
            self.getStatus()
            if tm.monotonic() - t > 2.0 :
                print("checkRotateFlag time over: flag=", gbv.rotate_flag)
                return
            
    def checkMoveFlag(self):
        t = tm.monotonic()
        while gbv.move_flag == 1:
            ret,gbv.frame = self.cap.read()
            print("move_flag = ", gbv.move_flag)
            self.getStatus()
            if tm.monotonic() - t > 1.0 :
                return None
        return True

    def checkStandUp(self):
        print("check stand up")
        gbv.start_flag = 0
        while True:
            while gbv.start_flag == 0:
                ret,gbv.frame = self.cap.read()
                self.getStatus()
                tm.sleep(0.1)
                if gbv.system_start == 0:
                    raise Exception("systen_off")

            t = tm.monotonic()
            while gbv.start_flag == 1:
                ret,gbv.frame = self.cap.read()
                self.getStatus()
                tm.sleep(0.1)
                if tm.monotonic() - t > 1.0 :  #Check if sytem  is stable
                    return
                if gbv.system_start == 0:
                    raise Exception("systen_off")


############################################################################
## distance adjustment
############################################################################import math
"""
import math
import colorDetection as cD 
centerX, centerY = 640, 360
H_viewAngle, V_viewAngle = 48/2, 28/2
robotHight = 170
adjustment = 0.0  #bigger is longer
Vth = 105.0 #106.0
"""
import cv2

def calcD(color, pos, centerX, centerY, targetHight):
    x, y, maxarea, ECx, ECy, ECr = (pos)
    cx = centerX - x
    hth = H_viewAngle * (cx/centerX) #[degree]
                
    cy = y - centerY
    vth = V_viewAngle * (cy/centerY)
    vtemp = (180 - (Vth + vth ) + gbv.adjustment - gbv.Angle) 
    distance = (robotHight - targetHight) * np.tan(np.radians(vtemp))
    distance += 40.0
    #print("[%s] x=%0.2f y=%0.2f distance=%0.2f[mm] vtemp=%0.2f[deg] fAngle=%0.2f"
    #      %(color, x, y, distance, vtemp, gbv.Angle))
    return distance, hth

def calibrate(distance, pos, centerX, centerY, targetHight):
    print("calibration")
    x, y, maxarea, ECx, ECy, ECr = (pos)
    #cx = centerX - x
    #hth = H_viewAngle * (cx/centerX) #[degree]               
    cy = y - centerY
    vth = V_viewAngle * (cy/centerY)
    
    distance -= 40.0
    vtemp = np.degrees(np.arctan(distance/robotHight))
    gbv.adjustment =  vtemp - (180 - (Vth + vth) - gbv.Angle)
    return
    
def distanceAdjust(cap, csbr, ser, centerX, centerY, ballHeight):
    
    while True:
        
        csbr.getData()
        posRed = cD.find_color(cap, "RED")    #Ball 
        posGr = cD.find_color(cap, "GREEN")   #Green Goal
        cv2.line(gbv.tempFrame, (0, 360), (1280, 360), (255, 255, 255), thickness=1, lineType=cv2.LINE_4)
        cv2.line(gbv.tempFrame, (640, 0), (640, 720), (255, 255, 255), thickness=1, lineType=cv2.LINE_4)
        
        if posRed is not None:
            x, y, maxarea, ECx, ECy, ECr = (posRed)
            gbv.tempFrame = cv2.circle(gbv.tempFrame,(int(ECx), int(ECy)),int(ECr*1.2),(0, 0, 255), 5)
            cv2.arrowedLine(gbv.tempFrame,(640,720),(int(ECx),int(ECy)),(0,0,255),thickness=2,tipLength=0.08)
            distance, hthRed = calcD("RED", posRed, centerX, centerY, ballHeight)
            dis=("%0.0fmm" %(distance))
            cv2.putText(gbv.tempFrame, dis ,(0,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255),2)
            th=("%0.1fdegree" %(hthRed))
            cv2.putText(gbv.tempFrame, th ,(150,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255),2)
            #print("posRed=", posRed)
        
        if posGr is not None:
            x, y, maxarea, ECx, ECy, ECr = (posGr)
            gbv.tempFrame = cv2.circle(gbv.tempFrame,(int(ECx), int(ECy)),int(ECr*1.2),(0, 255, 0), 5)
            cv2.arrowedLine(gbv.tempFrame,(640,720),(int(ECx),int(ECy)),(0,255,0),thickness=2,tipLength=0.08)
            distance, hthGr = calcD("GREEN", posGr, centerX, centerY, 0)
            dis=("%0.0fmm" %(distance))
            cv2.putText(gbv.tempFrame, dis ,(0,100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0),2)
            th=("%0.1fdegree" %(hthGr))
            cv2.putText(gbv.tempFrame, th ,(150,100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0),2)
            #print("posGr=", posGr)
        
        gbv.frame = gbv.tempFrame
        cv2.imshow('frame',gbv.frame)

        key = cv2.waitKey(500) & 0xFF
        
        if gbv.adjustIn != 0 :
            print("adjustIn", gbv.adjustIn)
            if gbv.adjustIn == "":
                gbv.adjustIn = 0
                break
            else:
                inD = float(gbv.adjustIn)
                calibrate(inD, posGr, centerX, centerY, 0)
                print("adjustment = %0.2f" %(gbv.adjustment))
                gbv.adjustIn = 0
                
        """    
        if posGr is not None and cv2.waitKey(1000) & 0xFF == 32:  #SP
            #cap.release()
            #cv2.destroyAllWindows()
            
            inD = input("input GREEN length: ")
            if inD == "":
                print("no adjustment")
                break
            else:
                inD = float(inD)
            calibrate(inD, posGr, centerX, centerY, 0)
            print("adjustment = %0.2f" %(gbv.adjustment))
         """   