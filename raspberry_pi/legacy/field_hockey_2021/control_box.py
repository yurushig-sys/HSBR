# -*- coding: utf-8 -*-

import serial
import time
import pyautogui
import tkinter as tk
import tkinter.scrolledtext as tksc
import threading

import controlArduino as cAd
import globalVariables as gbv

class ControlBox(threading.Thread):
    
    #Constractor
    def __init__(self, ser, cap):
        
        self.ser = ser
        #self.cms = cms
        self.cap = cap
        self.P_P = 10.0
        self.P_I = 0.001
        self.P_D = 10.0
        self.TPy = 0.0
        self.csbr = cAd.controleSBR(ser, self.cap)
        
        self.root = tk.Tk()
        self.root.geometry("1000x560+920+0")
        self.root.title("SBR Controller")
        
        self.frame1 = tk.Frame(self.root, bd=2, relief="flat")
        self.frame2 = tk.Frame(self.root, bd=2, relief="flat")
        self.frame3 = tk.Frame(self.root, bd=2, relief="sunken")
        self.frame4 = tk.Frame(self.root, bd=2, relief="solid")

        self.frame1.pack(fill="x")
        self.frame2.pack(fill="x")
        self.frame3.pack(fill="x")
        self.frame4.pack(fill="x")
        
        self.cmd_in()
        self.cmd_popup()
        #self.data_disp()
        self.monitor()
        self.com_start()
        
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.root.mainloop()

# command input
    def cmd_in(self):
        self.label1 = tk.Label(self.frame1, text='Command Input')
        self.label1.pack(side ="left")
        self.en = tk.Entry(self.frame1, width=40)
        self.en.pack(side ="left")
        self.en.focus_set()
        self.en.bind("<Return>", self.get_cmd)
        self.bt1 = tk.Button(self.frame1, text="click", command=self.clicked2)
        self.bt1.pack(side="left")
        
        self.label2 = tk.Label(self.frame2, text='Command =  ')
        self.label2.grid(row=0, column=0)
    
        self.bt_log_on = tk.Button(self.frame2, text="log on ", command=self.log_on)
        self.bt_log_off = tk.Button(self.frame2, text="log off", command=self.log_off)
        self.bt_start = tk.Button(self.frame2, text="system start", command=self.sys_start)
        self.bt_stop = tk.Button(self.frame2, text="system stop", command=self.sys_stop)
        self.bt_releaseBp = tk.Button(self.frame2, text="release pBp", command=self.releaseBp)
        self.bt_log_on.grid(row=1, column=0)
        self.bt_log_off.grid(row=1, column=1)
        self.bt_start.grid(row=1, column=3)
        self.bt_stop.grid(row=1, column=4)
        self.bt_releaseBp.grid(row=1, column=6)

    def get_cmd(self, event):
        self.clicked2()
            
    def clicked2(self):
        self.CMD = self.en.get()
        self.en.delete(0, tk.END)
        self.label2["text"] = "Command = " +self.CMD
        print(self.CMD)
        print("adjustMode=", gbv.adjustMode)
        if gbv.adjustMode == 1:
            gbv.adjustIn = self.CMD
            print("adjust input", gbv.adjustIn)
        self.ser.write(str.encode(str(self.CMD) + '\n'))   
 
    def log_on(self):
        self.csbr.logOn()
        
    def log_off(self):
        self.csbr.logOff()

    def sys_start(self):
        self.csbr.initServoMotors() 
        self.csbr.standUp()
        gbv.system_start = 1
        #self.cms.initialPosition()
        
    def sys_stop(self):
        self.csbr.initServoMotors() 
        self.csbr.turnDown()
        print("system stop")
        gbv.system_start = 0
        
    def releaseBp(self):
        #print("release BP")
        gbv.pBp_flag = 1
       
        
# popup window for command input
    def cmd_popup(self):
        
        self.frame2.bind('<Button-3>', self.showPopup)

    def showPopup(self, event):
        menu_top = tk.Menu(tearoff=False)
        menu_2nd = tk.Menu(menu_top,tearoff=0)
        menu_3rd = tk.Menu(menu_2nd,tearoff=0)

        menu_top.add_command(label='Taget position Y', command=self.tpy)
        menu_top.add_separator()
        menu_top.add_command(label='Position PID',underline=5,command=self.p_pid)
        menu_top.add_cascade (label='test1', menu=menu_2nd,under=5)

        menu_2nd.add_command(label='test2-1',under=4)
        menu_2nd.add_cascade(label='test2-2',under=5,menu=menu_3rd)

        menu_3rd.add_command(label='test3-1',under=11)
        menu_3rd.add_command(label='test3-2',under=8)

        menu_top.post(event.x_root,event.y_root)        

    def tpy(self):
        cx,cy = pyautogui.position() 
        self.win = tk.Toplevel()
        self.win.title("Target Position")
        self.win.geometry('+'+str(cx)+'+'+str(cy))
        
        l1=tk.Label(self.win, text="Input Target Posion Y ")
        self.sptpy = tk.StringVar()
        self.sptpy.set(self.TPy)
        self.spb = tk.Spinbox(self.win,textvariable=self.sptpy,from_=-10.0, to=10.0,increment=0.5)
        b=tk.Button(self.win, text="Okay", command=self.get_tpy) 

        l1.grid(row=0, column=0)
        self.spb.grid(row=0,column=1)
        b.grid(row=0,column=2)
        
    def get_tpy(self):
        self.TPy = self.spb.get()
        self.win.destroy()
        print("TPy={}".format(self.TPy))
        self.ser.write(str.encode("TPy:"+self.TPy+"\n"))

    def p_pid(self):
        cx,cy = pyautogui.position() 
        self.win = tk.Toplevel()
        self.win.geometry('+'+str(cx)+'+'+str(cy)) 
        
        l1=tk.Label(self.win, text="Position P ")
        self.sptxt1 = tk.StringVar()
        self.sptxt1.set(self.P_P)
        sp1 = tk.Spinbox(self.win,textvariable=self.sptxt1,from_=0.0,to=50.0,increment=0.5)
        l2=tk.Label(self.win, text="Position I ")
        self.sptxt2 = tk.StringVar()
        self.sptxt2.set(self.P_I)
        sp2 = tk.Spinbox(self.win,textvariable=self.sptxt2,from_=0,to=0.01,increment=0.001)
        l3=tk.Label(self.win, text="Position I ")
        self.sptxt3 = tk.StringVar()
        self.sptxt3.set(self.P_D)
        sp3 = tk.Spinbox(self.win,textvariable=self.sptxt3,from_=-50,to=50.0,increment=0.5)
        
        b=tk.Button(self.win, text="Okay", command=self.get_parameter)   #command=win.destroy)
        
        l1.grid(row=0, column=0)
        sp1.grid(row=0,column=1)
        l2.grid(row=1, column=0)
        sp2.grid(row=1,column=1)
        l3.grid(row=2, column=0)
        sp3.grid(row=2,column=1)
        b.grid(row=3, column=1)

    def get_parameter(self):
        self.P_P = self.sptxt1.get()
        self.P_I = self.sptxt2.get()
        self.P_D = self.sptxt3.get()
        self.win.destroy()
        print("P={} I={} D={}".format(self.P_P, self.P_I, self.P_D))
        self.ser.write(str.encode("P_P:"+self.P_P+"\n"))
        self.ser.write(str.encode("P_I:"+self.P_I+"\n"))
        self.ser.write(str.encode("P_D:"+self.P_D+"\n"))



# monitor display    
    def monitor(self):
        self.sctx = tksc.ScrolledText(self.frame3,font=("Helvetica", 14), height=15 )
        self.sctx.pack(fill='x')
        self.bt2 = tk.Button(self.frame3, text="clear", command=self.clicked1)
        self.bt2.pack(side="right", padx=20)
        
        self.bln = tk.BooleanVar()
        self.bln.set(True)
        self.chk = tk.Checkbutton(self.frame3, variable=self.bln, text='Auto Scroll')
        self.chk.pack(sid="left")
        
    def clicked1(self):
        #self.sctx.insert('end', self.CMD + '\n')
        self.sctx.delete('1.0', 'end')

# monitor input
    def com_start(self):
        th=threading.Thread(target=self.com_receive)
        th.start()

    def com_receive(self):
        
        while True:
            data = self.ser.readline()    #read line from arduino
            if len(data) > 0:         #if timeover 0.1s then len(data) == 0
                #self.sctx.insert('end', data.rstrip().decode())
                data = data.decode()
                self.sctx.insert('end', data)
                if self.bln.get():
                    self.sctx.see('end')               
                
                if data[0:4] == 'Data' or data[0:6] == 'Status':
                    self.analyze_data(data)
    
 
    def analyze_data(self, data):
        param = []
        l = len(data)
        m = 0
        n = 0
        while n < l :
            n = data.find(" ", m)
            if n < 0:
                param.append(data[m:l])
                break
            
            if n > m :                  #if n = m, skip
                param.append(data[m:n])
                #print(l, m, n, param)
            m = n + 1
        
        if param[0] == "Status":
            gbv.start_flag = int(param[1])
            gbv.move_flag = int(param[2])
            gbv.rotate_flag = int(param[3])
            gbv.continuous_flag = int(param[4])
            gbv.paramIndex = int(param[5])
            #print('Flags: start=%d move=%d rotate=%d cont=%d'
            #  %(gbv.start_flag, gbv.move_flag, gbv.rotate_flag, gbv.continuous_flag)) 
        
        elif param[0] == "Data":
            gbv.Wx = float(param[1])
            gbv.Wy = float(param[2])
            gbv.Lp = float(param[3])
            gbv.TLp = float(param[4])
            gbv.Theta = float(param[5])
            gbv.Angle = float(param[6])
            gbv.Battery_voltage = float(param[7])
            #print('Wx=%0.2f Wy=%0.2f Lp=%0.2f TLp=%0.2f Theta=%0.2f Angle=%0.2f Battery=%0.2f'
            #  %(gbv.Wx, gbv.Wy, gbv.Lp, gbv.TLp, gbv.Theta, gbv.Angle, gbv.Battery_voltage))
            
        gbv.que_flag = 0

        
class pseudoBreakPoint():

    def breakPoint(self,comment, cap, csbr):
        print("pseudo Break Point ", comment)
        gbv.pBp_flag = 0
        while gbv.pBp_flag == 0 :   #wait unti system start
            #ret,gbv.frame = cap.read()
            pass
        csbr.getStatus() #if system is down, make exception

