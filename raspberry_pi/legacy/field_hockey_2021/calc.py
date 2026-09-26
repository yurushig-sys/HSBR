# -*- coding: utf-8 -*-
#
# function calculation
#

import numpy as np
import cv2
import globalVariables as gbv

#2点の座標、(x0, y0)から(x1, y1)方向の角度を求める
def angle(x0,y0,x1, y1):
    if y1-y0 == 0:
        ang=np.radians(90.0)
    else:
        ang=np.arctan(abs(y1 - y0)/abs(x1 - x0)) #0～90°

    if y1 - y0 >= 0:
        if x1 - x0 < 0:
            ang = np.pi - ang      #90～180°
    else:
        if x1 - x0 < 0:
            ang += np.pi        #180～270°
        else:
            ang = 2 * np.pi - ang    #270～360°

    return ang  #radians 0～2PI

#orbit calculation
#


r1 = 100 #150 #150
R2 = 200 #150 #200 #300

def calcOrbit(Rx, Ry, Gx, L1, Th1):
    
    r2 = R2
    
    L1 = round(L1)
    if L1 <= 2*r2+r1:
        r2 = round((L1 - r1)/2) - 1
    
    if Th1 < 0:
        Th1 += 2*np.pi

    #ボールまでの距離と角度を基に、ボールの座標を算出
    Bx = round(L1*np.cos(Th1),2) + Rx
    By = round(L1*np.sin(Th1),2) + Ry
    #print("Bx=%0.2f  By=%0.2f" %(Bx, By))

    #(Bx, By)からゴールまで直線の方程式 y=ax + b
    a=-By/(Gx-Bx)
    b=Bx*By/(Gx-Bx)+By

    #ボールがロボットの上にあるか、下にあるかの判定
    #直線Plにおいてx=Rxの時、y≧Ryならロボットは上、y<Ryなら下
    yRx = a*Rx+b+r1*np.sqrt(a**2+1)
    if (yRx >= Ry) :
        judge = 1
    else:
        judge = -1
    #print("judge = ", judge)

    #y=ax + bに垂直で（Bx,By)を通る直線(Vl)の方程式　y-By = -1/a・(x-Bx)
    #y=-x/a+Bx/a+By　➡　y=cx+d
    if a==0:
        c=0
        d=By
    else:
        c=-1/a
        d=Bx/a+By
    #Vlがx軸と交わる点の座標と角度
    vls = (a*By+Bx, 0)
    (vx, vy)=vls
    if vx == Bx and vy == By:
        Thv=np.radians(90)
    else:
        Thv = angle(vx, vy, Bx, By)
    if judge == 1 and By < 0:
        Thv -=np.pi
    #print("Thv=%0.2f" %(np.degrees(Thv)))

    #Vl上において、(Bx,By)から距離R離れた点の座標（TCx, TCy）を求める。
    if (judge == 1):
        R=r2-r1
    else:
        R=r2+r1   

    if By == 0:
        Tx=Bx
        Ty=-R
    else:
        if judge == 1:  #ボールはロボットより上　∴Ty<Byでなければならない
            Tx=Bx - R/np.sqrt(c**2+1)
            Ty=c*Tx+d
            if Ty > By:
                Tx=Bx + R/np.sqrt(c**2+1)
                Ty=c*Tx+d
        else:           #ボールはロボットより上　∴Ty＞Byでなければならない　
            Tx=Bx - R/np.sqrt(c**2+1) 
            Ty=c*Tx+d
            if Ty < By:
                Tx=Bx + R/np.sqrt(c**2+1)
                Ty=c*Tx+d
            
    #print("Tx=%0.2f Ty=%0.2f" %(Tx, Ty))            

    #(Rx, Ry)と(Tx, Ty)間の直線L2の長さとx軸との間の角度θ2を求める
    L2 = np.sqrt((Tx-Rx)**2 + (Ty-Ry)**2)
    Th2 = angle(Rx,Ry,Tx,Ty)  #(0～360°)
    #print("L2=%0.2f Th2=%0.2f" %(L2, np.degrees(Th2)))

    #(Rx, Ry)から(Tx, Ty)を中心とする半径r2の円に接する直線の長さL3を求める
    L3 = np.sqrt(L2**2 - r2**2)
    #L3とr2は直角ゆえ、L2 – L3間の角度θ23は
    Th23 = np.arcsin(r2/L2)     #(0～90°)

    #L3とx軸間の角度θ3はボールとロボットの位置関係により
    if judge == 1:
        Th3 = Th2 + Th23
    else :
        Th3 = Th2 - Th23
    
    #print("L3=%0.2f Th23=%0.2f Th3=%0.2f" %(L3, np.degrees(Th23),np.degrees(Th3)))

    # (Px, Py)を求める
    Px = L3 * np.cos(Th3) + Rx
    Py = L3 * np.sin(Th3) + Ry
    print("L1=%0.2f L2=%0.2f L3=%0.3f" %(L1, L2, L3))
    print("Bx=%0.2f By=%0.2f Tx=%0.2f Ty=%0.2f Px=%0.2f Py=%0.2f" %(Bx, By, Tx, Ty, Px, Py))

    #直線 (Px, Py)-(Tx, Ty)とx軸間の角度θ5を求める
    Th5 = angle(Tx,Ty,Px,Py)  #(0～360°)

    #ロボットの第一段回転角度を求める　
    Th4 = Th3 - Th1    #θ4>0：左旋回、　θ4<0：右旋回
    if Th4 > 2*np.pi:
        Th4 -= 2*np.pi
    if Th4 > np.pi:
        Th4 -= 2*np.pi
    elif Th4 < -np.pi:
        Th4 += 2*np.pi

    #(Px, Py) – (Tx,, Ty) とx軸の角度θ5  (0~360°）を求める
    Th5 = angle(Tx, Ty, Px, Py)

    #(Px, Py) からヒッティングポイントまでの回転角 θ6を求める
    Th6 = Thv - Th5    #θ6>0：左旋回、　θ6<0：右旋回
    print("Th5=%0.2f Thv=%0.2f Th6=%0.2f" %(np.degrees(Th5), np.degrees(Thv), np.degrees(Th6)))
    #print("Th4=%0.2f L3=%0.2f Th6=%0.2f" %(np.degrees(Th4), L3, np.degrees(Th6)))
    return Th4, L3, Th6, r2


