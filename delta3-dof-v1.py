
import timeit
import math
import time
import serial
import numpy as np
import struct
from numpy import linalg as LA
import matplotlib.pyplot as plt

#%%
class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            
        return self.output;
        

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

    def setSetPoint(self, set_point):
        self.SetPoint = set_point



#%%
# configure the serial connections (the parameters differs on the device you are connecting to)
#ser4 = serial.Serial(port='/dev/ttyUSB0',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=None, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None) 
#ser2 = serial.Serial(port='/dev/ttyUSB1',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=None, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None)
#ser3 = serial.Serial(port='/dev/ttyUSB2',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=None, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None) 
#ser4 = serial.Serial(port='/dev/ttyUSB3',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=None, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None) 

'''
ser4 --> motor 1
ser2 --> motor 2
ser3 --> motor 3
'''
ser4 = serial.Serial(port='COM10',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=.005, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None) 
ser2 = serial.Serial(port='COM11',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=.005, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None)
ser3 = serial.Serial(port='COM12',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=.005, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None) 
#ser4 = serial.Serial(port='COM6',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=.005, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None) 

#ser = serial.Serial(port='COM7',baudrate=9600,timeout=.005) 

#print("port1 open =",ser4.isOpen(),"port2 open =",ser2.isOpen(),"port3 open =",ser3.isOpen(),"port4 open =",ser4.isOpen())

#%%
def CHKS_Kinco(value) :

    value =  -1*sum(value)
    value = 2**32 + value 
    C = list(struct.unpack('>4B',struct.pack('>L',value)))
    return C[3]

def HexCon(value) :
    if value < 0 :
        value = 2**32 + value 
    
    H = list(struct.unpack('>4B',struct.pack('>L',value)))
    
    return H


#def Position(ID,position,profile_speed): #in degree
#    ser4.flushInput()
#
#    position = position*10000/360 * 50
##    print("position =",position)
#    value_Hex = HexCon(int(position))
#    s = [ID,35,0x7A,96,0,int(value_Hex[3]),int(value_Hex[2]),int(value_Hex[1]),int(value_Hex[0])]
#    s.append(CHKS_Kinco(s))
#    s = bytearray(s)
#    ser4.write(s)
#    
#    ser4.flushInput()
#
#    if profile_speed > 20:
#        profile_speed = 20
#    if profile_speed < -20:
#        profile_speed = -20
#    
#    if (ID==1 or ID==3) :
#        profile_speed = profile_speed * -1
#    profile_speed = math.floor(profile_speed*2730.66 * 50)     
#    value_Hex = HexCon(profile_speed)
#    s = [ID,35,0x81,96,0,int(value_Hex[3]),int(value_Hex[2]),int(value_Hex[1]),int(value_Hex[0])]
#    s.append(CHKS_Kinco(s))
#    s = bytearray(s)
#    ser4.write(s)
    

def Drive_Enable(ID,value) :

    if value == 0:
        value = 6
    elif value == 1:
        value = 0x2F     # Enable Driver
    elif value == 2:
        value = 0x103F   # Start Absolute  position
    elif value == 3:
        value = 0x105F   # Start Relative position
    elif value == 4:
        value = 0x0F     # Start Torque and Speed mode
        
    value_Hex = HexCon(value)
    s=[ID,43,0x40,96,0,int(value_Hex[3]),int(value_Hex[2]),int(value_Hex[1]),int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1 : 
        ser4.write(s)
    elif ID == 2 : 
        ser2.write(s)
    elif ID == 3 : 
        ser3.write(s)


def Write_reg(ID,value,CMD,index1,index2,Subindex):
    value_Hex = HexCon(value)
    s = [ID,CMD,index1,index2,Subindex,int(value_Hex[3]),int(value_Hex[2]),int(value_Hex[1]),int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
#    ser4.flushInput()
    if ID == 1 : 
        ser4.write(s)
    elif ID == 2 : 
        ser2.write(s)
    elif ID == 3 : 
        ser3.write(s)

        
#-------------------------------Target_Toruqe---------------------------------
def Set_Torque_Nm(ID,Target_Torque_Nm):
    ser4.flushInput()
    
    Target_Torque_Dec = math.floor(Target_Torque_Nm/2.668)*10
    value_Hex = HexCon(Target_Torque_Dec)
    s = [ID,43,113,96,0,int(value_Hex[3]),int(value_Hex[2]),int(value_Hex[1]),int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1 : 
        ser4.write(s)
    elif ID == 2 : 
        ser2.write(s)
    elif ID == 3 : 
        ser3.write(s)



#def Set_Max_Speed_rpm(ID,Speed_rpm):
#    ser4.flushInput()
#    ser2.flushInput()
#    ser3.flushInput()
#    ser4.flushInput()
##    if Speed_rpm > 1000:
##        Speed_rpm = 1000
#    
#    Speed_Dec = math.floor(Speed_rpm*2730.66) 
#    value_Hex = HexCon(Speed_Dec)
#    s = [ID,43,115,96,0,int(value_Hex[3]),int(value_Hex[2]),int(value_Hex[1]),int(value_Hex[0])]
#    s.append(CHKS_Kinco(s))
#    s = bytearray(s)
#    if ID == 1:
#        ser4.write(s)
#    elif ID == 2:
#        ser2.write(s)
#    elif ID == 3:
#        ser3.write(s)
#--------------------------------Operation_Mode-------------------------------
def Operation_Mode(ID,value):
    ser4.flushInput()
#    speed = -3,3 -----torque = 4 ---- pulse = -4  ,position=1
    value_Hex = HexCon(value)
    s = [ID,47,96,96,0,int(value_Hex[3]),int(value_Hex[2]),int(value_Hex[1]),int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1 : 
        ser4.write(s)
    elif ID == 2 : 
        ser2.write(s)
    elif ID == 3 : 
        ser3.write(s)



#--------------------------------------speed_rpm---------------------------------------------
def Set_Speed_rpm(ID,Speed_rpm):
    if Speed_rpm > 10:
        Speed_rpm = 10
    if Speed_rpm < -10:
        Speed_rpm = -10
#    if (ID==1 or ID==3) :
#        Speed_rpm = Speed_rpm * -1
    Speed_Dec = math.floor(Speed_rpm*2730.66 * 50) 
    value_Hex = HexCon(Speed_Dec)
    s = [ID,35,255,96,0,int(value_Hex[3]),int(value_Hex[2]),int(value_Hex[1]),int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1 : 
        ser4.write(s)
    elif ID == 2 : 
        ser2.write(s)
    elif ID == 3 : 
        ser3.write(s)
        
#-------------------------------Pos_Actual-----------------------------------
def Pos_Actual(ID):
#    if ID==1:
#        ser4.flushInput()
#    elif ID==2:
#        ser2.flushInput()
#    elif ID==3:
#        ser3.flushInput()
#    elif ID==4:
#        ser4.flushInput()
#    ser4.flushInput()
#    if ID==2:
#        time.sleep(0.002)
    s = [ID,64,99,96,0,0,0,0,0]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
#    print("s = ",list(s))

#        print(ser4.in_waiting)
#        if ser4.in_waiting:
#            ser4.flushInput()
    count=0
    if ID == 1 : 
        ser4.flushInput()
        ser4.write(s)
        pos1 = list(ser4.read(10))
        while len(pos1)!=10:
            pos1 = list(ser4.read(10))
            count=count+1
            print("{}times tried to read pos but failed from ID={}".format(count,ID))
            if count==40:
                print("Unable to read data from ID={}".format(ID))
                break
#        print(pos1)
#        pos1 = list(ser4.read(10))
    elif ID == 2 : 
        ser2.flushInput()
        ser2.write(s)
        pos1 = list(ser2.read(10))
        while len(pos1)!=10:
            pos1 = list(ser2.read(10))
            count=count+1
            print("{}times tried to read pos but failed from ID={}".format(count,ID))
            if count==40:
                print("Unable to read data from ID={}".format(ID))
                break
#        time.sleep(0.02)
#        print(pos1)
#        pos1 = list(ser4.read(10))
    elif ID == 3 : 
        ser3.flushInput()
        ser3.write(s)
        pos1 = list(ser3.read(10))
        while len(pos1)!=10:
            pos1 = list(ser3.read(10))
            count=count+1
            print("{}times tried to read pos but failed from ID={}".format(count,ID))
            if count==40:
                print("Unable to read data from ID={}".format(ID))
                break
#        pos1 = list(ser3.read(10))

        
#        time.sleep(0.1)
#    pos1 = list(ser4.read(10))
#    print(pos1)
#    pos1 = list(ser4.read(10))
#    print(pos1)
#    while (len(pos1)!=10 or pos1[1]!= 67 or pos1[2]!= 99 or pos1[3]!= 96):
    o=0
    if (len(pos1)!=10 ):
#        print("pos1 =", pos1,"--",ID)
        return -333333
    else:
        p = pos1 +  pos1
        template=[ID,67,99,96]
        for t in range(len(p)//2 + 1):
            if template == p[t:t+len(template)]:
                pos1= p[t:t+10]
#                print("$pos1 =", pos1,"--",ID)
                o=1
                break
    if o==0:
#        print("pos1 =", pos1,"--",ID)
        return -333333
#            print("pos1 =", pos1,"--",ID)
#            return -333333
#        print("pos1 old =", pos1)
#        pos1 = list(ser4.read(10))
    #print("pos1 =", pos1)
    pos=value_read(pos1)
    while abs(pos) > 5000 :
        return -333333
#    if ID ==1 or ID==3:
#        pos = pos*-1
    return ( pos / 50)  
        


def Velocity_Actual(ID):
    
    s = [ID,64,108,96,0,0,0,0,0]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
#    print("s = ",list(s))
#---------------------------------------------------------------      
           
    if ID == 1 : 
        ser4.flushInput()
        ser4.write(s)
        vel = list(ser4.read(10))
    elif ID == 2 : 
        ser2.flushInput()
        ser2.write(s)
        vel = list(ser2.read(10))
    elif ID == 3 : 
        ser3.flushInput()
        ser3.write(s)
        vel = list(ser3.read(10))

#        time.sleep(0.1)
    
#    print("vel =", vel,"--",ID)
    o=0
#    while (len(vel)!=10 or vel[1]!= 67 or vel[2]!= 108 or vel[3]!= 96):
    if (len(vel)!=10):
#        print("pos1 =", pos1,"--",ID)
        return -666666
    else:
        p = vel +  vel
        template=[ID,67,108,96]
        for t in range(len(p)//2 + 1):
            if template == p[t:t+len(template)]:
                vel= p[t:t+10]
#                print("$pos1 =", pos1,"--",ID)
                o=1
                break
    if o==0:
#        print("pos1 =", pos1,"--",ID)
        return -666666
#        print("pos1 old =", pos1)
#        pos1 = list(ser4.read(10))
    #print("pos1 =", pos1)
    vel=value_read_vel(vel)
    while abs(vel) > 3000 :
        return -666666
#    if ID ==1 or ID==3:
#        vel = vel*-1
#    print("vel1 old =", vel1)
#        vel1 = list(ser4.read(10))
#        print("vel1 =", vel1)
    return vel/ (50)
  

def Torque_Actual(ID):
#    ser4.flushInput()
    s = [ID,64,119,96,0,0,0,0,0]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
#    print("s = ",list(s))
#---------------------------------------------------------------      
    if ID == 1:
#        print(ser4.in_waiting)
#        if ser4.in_waiting:
#            ser4.flushInput()
            
        ser4.write(s)
#        time.sleep(0.1)
        torque1 = list(ser4.read(10))
#        print("torque1 old =", torque1)
#        torque1 = list(ser4.read(10))
#        print("torque1 =", torque1)
#        return value_read_torque(torque1)
        
    elif ID == 2:
#        print(ser2.in_waiting)
#        if ser2.in_waiting:
#            ser2.flushInput()
            
        ser2.write(s)
#        time.sleep(0.1)
        torque1 = list(ser2.read(10))
#        print("torque2 old =", torque2)
#        torque2 = list(ser4.read(10))
#        print("torque2 =", torque2)
#        return value_read_torque(torque2)
        
    elif ID == 3:
#        print(ser3.in_waiting)
#        if ser3.in_waiting:
#            ser3.flushInput()
            
        ser3.write(s)
#        time.sleep(0.1)
        torque1 = list(ser3.read(10))
#        print("torque3 old =", torque3)
#        torque3 = list(ser4.read(10))
#        print("torque3 =", torque3)
#        return value_read_torque(torque3)
        
        
    o=0
    if (len(torque1)!=10 ):
#        print("pos1 =", pos1,"--",ID)
        return -333333
    else:
        p = torque1 +  torque1
        template=[ID,67,119,96]
        for t in range(len(p)//2 + 1):
            if template == p[t:t+len(template)]:
                torque1= p[t:t+10]
#                print("$pos1 =", pos1,"--",ID)
                o=1
                break
    if o==0:
#        print("pos1 =", pos1,"--",ID)
        return -333333
#            print("pos1 =", pos1,"--",ID)
#            return -333333
#        print("pos1 old =", pos1)
#        pos1 = list(ser4.read(10))
    #print("pos1 =", pos1)
    torque=value_read(torque1)
    while abs(torque) > 5000 :
        return -333333
#    if ID ==1 or ID==3:
#        pos = pos*-1

    return value_read_torque(torque)
        

        
 
def value_read(value):
    Pos = value[8]*(2**24) + value[7]*(2**16) + value[6]*(2**8) + value[5]
    if value[8] >= 255:
        Pos = -2**32 + Pos
    Pos = ((Pos/10000)*360)
#    print("Pos =",Pos)
    return Pos

def value_read_vel(value):
    Vel = value[8]*(2**24) + value[7]*(2**16) + value[6]*(2**8) + value[5]
    if value[8] >= 255:
        Vel = -2**32 + Vel
    Vel = Vel/2730.66
#    print("Vel_rpm =",Vel)
    return Vel
    
def value_read_torque(value):
    Torque = value[8]*(2**24) + value[7]*(2**16) + value[6]*(2**8) + value[5]
    if value[8] >= 255:
        Torque = -2**32 + Torque
    Torque = (Torque/1000)*2.668
    return Torque
#    print("Torque_Nm =",Torque)


Write_reg(1,800,35,0x83,96,0)   #Acceleration
Write_reg(1,800,35,0x84,96,0)   #Deceleration        
Write_reg(2,800,35,0x83,96,0)   #Acceleration
Write_reg(2,800,35,0x84,96,0)   #Deceleration
Write_reg(3,800,35,0x83,96,0)   #Acceleration
Write_reg(3,800,35,0x84,96,0)   #Deceleration        
  #Deceleration


Write_reg(1,1,35,0x7D,96,1)   #Soft_positive_limit
Write_reg(1,1,35,0x7D,96,2)   #Soft_negative_limit        
Write_reg(2,1,35,0x7D,96,1)   #Soft_positive_limit
Write_reg(2,1,35,0x7D,96,2)   #Soft_negative_limit
Write_reg(3,1,35,0x7D,96,1)   #Soft_positive_limit
Write_reg(3,1,35,0x7D,96,2)   #Soft_negative_limit        


#Set_Max_Speed_rpm(1,40)
#Set_Max_Speed_rpm(2,40)
#Set_Max_Speed_rpm(3,40)
#Set_Max_Speed_rpm(4,40)


#Write_reg(1,15,43,115,96,0)
#Write_reg(2,15,43,115,96,0)
#Write_reg(3,15,43,115,96,0)
#Write_reg(4,15,43,115,96,0)
def Position_read():
    
    print("pp1=",Pos_Actual_r(1))
#    time.sleep(.01)
    print("pp2=",Pos_Actual_r(2))
#    time.sleep(.01)  
    print("pp3=",Pos_Actual_r(3))
#    time.sleep(.008)  

    
def Stop():
    Set_Speed_rpm(1,0)
     
    Set_Speed_rpm(2,0)
       
    Set_Speed_rpm(3,0)

global offset
#offset = None             
def Offset():
    global offset
    p1 = Pos_Actual_r(1)
    p2 = Pos_Actual_r(2)
    p3 = Pos_Actual_r(3)
    offset=[p1, p2, p3]
#offset=[0, 0, 0]
def Pos_Actual_r(ID):
    global offset
    if 'offset' not in globals():
        offset=[0, 0, 0]    
#    start=timeit.default_timer()
    pp = Pos_Actual(ID)
#    print("__",timeit.default_timer()-start)
    i=0
#    start=timeit.default_timer()
    while pp == -333333  :
#        if ID==1:
#            ser4.flushOutput()
#        elif ID==2:
#            ser2.flushOutput()
#        elif ID==3:
#            ser3.flushOutput()
#        elif ID==4:
#            ser4.flushOutput()
#        time.sleep(1)    
        pp = Pos_Actual(ID)
        i += 1
#        print(i,ID)
        if i >20:
            Stop()
            Disable_all()
            break
#    print("__**__",timeit.default_timer()-start)
    return pp - offset[ID-1]

def Velocity_Actual_r(ID):
    #start=timeit.default_timer()
    vel = Velocity_Actual(ID)
    #print("__",timeit.default_timer()-start)
    i=0
    while vel == -666666  :
        vel = Velocity_Actual(ID)
        i += 1
#        print(i,ID)
        if i >20:
            Stop()
            Disable_all()
            break
#    print(i)
    return vel

def Suction(s):
    ser.write(str(s).encode())
#define kp
def Set_kp_ki(kp,ki): 
    
    Write_reg(1,kp,43,0xF9,96,1)  
    Write_reg(2,kp,43,0xF9,96,1)     
    Write_reg(3,kp,43,0xF9,96,1)           
    
    Write_reg(1,ki,43,0xF9,96,2)       
    Write_reg(2,ki,43,0xF9,96,2)     
    Write_reg(3,ki,43,0xF9,96,2)      

     
#define spped_fb_n
def Set_fbn(value):     #defult = 7
    
    Write_reg(1,value,47,0xF9,96,5)       
    Write_reg(2,value,47,0xF9,96,5)     
    Write_reg(3,value,47,0xF9,96,5)      

def Set_kpp(kpp):
    
    Write_reg(1,kpp,43,0xFB,96,1)       
    Write_reg(2,kpp,43,0xFB,96,1)
    Write_reg(3,kpp,43,0xFB,96,1)        

def RS232_baudrate(b):   #540=19200 , 270=38400 , 90=115200      

    Write_reg(1,b,43,0xE0,0x2F,0)       
    Write_reg(2,b,43,0xE0,0x2F,0) 
    Write_reg(3,b,43,0xE0,0x2F,0)      

def Store_Loop_Data(s):   #store=1 , initial=10    

    Write_reg(1,s,47,0xF0,0x2F,1)       
    Write_reg(2,s,47,0xF0,0x2F,1)  
    Write_reg(3,s,47,0xF0,0x2F,1)       

def Close_Serial():
    ser4.close()
    ser2.close()
    ser3.close()
    ser.close()
     
def Motion_z(speed): #speed - == up
    
    Set_Speed_rpm(1,speed)     
    Set_Speed_rpm(2,speed)    
    Set_Speed_rpm(3,speed)    

    
def Motion_x(speed):
    
    Set_Speed_rpm(1,speed)     
    Set_Speed_rpm(2,speed)       
    Set_Speed_rpm(3,-1*speed)            
        
def Motion_y(speed):
    
    Set_Speed_rpm(1,-1*speed)     
    Set_Speed_rpm(2,speed)       
    Set_Speed_rpm(3,speed)                   
         
def Enable_all():    
    Drive_Enable(1,1)
#    time.sleep(.008)
    Drive_Enable(2,1)
#    time.sleep(.008)
    Drive_Enable(3,1)
#    time.sleep(.008)

    Operation_Mode(1,-3)
#    time.sleep(.008)
    Operation_Mode(2,-3)
#    time.sleep(.008)
    Operation_Mode(3,-3)
#    time.sleep(.008)
    Motion_z(0)
    Motion_z(0)

def Torque_Mode():
    Drive_Enable(1,4)
#    time.sleep(.008)
    Drive_Enable(2,4)
#    time.sleep(.008)
    Drive_Enable(3,4)
#    time.sleep(.008)

    Operation_Mode(1,4)
#    time.sleep(.008)
    Operation_Mode(2,4)
#    time.sleep(.008)
    Operation_Mode(3,4)
#    time.sleep(.008)
    Motion_z(0)
    Motion_z(0)
    
def Homing():
    position=[0,0,0]
    while -78-position[0]<0 or -78-position[1]<0 or -78-position[2]<0:
        if -78-position[0]<-10 or -78-position[1]<-10 or -78-position[2]<-10:
            Motion_z(-0.4)
            Motion_z(-0.4)
        else:    
            Motion_z(-0.2)
            Motion_z(-0.2)
        position[0]=Pos_Actual_r(1)
        position[1]=Pos_Actual_r(2)
        position[2]=Pos_Actual_r(3)
        if -78-position[0]>0 or -78-position[1]>0 or -78-position[2]>0:
            print("Arrived!")
            break
    Motion_z(0)
    Motion_z(0)
    Offset()

def Disable_all(): 
    Drive_Enable(1,0)
    time.sleep(.008)
    Drive_Enable(2,0)
    time.sleep(.008)
    Drive_Enable(3,0)
    time.sleep(.008)

e= (1/math.tan(np.deg2rad(30))) * 20
f= (1/math.tan(np.deg2rad(30))) * 26
re = 59.5
rf =  30.9

#s      = 165*2
sqrt3  = math.sqrt(3.0)
pi     = 3.141592653
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60  = sqrt3
sin30  = 0.5
tan30  = 1.0 / sqrt3

def Forward(theta1, theta2, theta3):
    x0 = 0.0
    y0 = 0.0
    z0 = 0.0
    
    t = (f-e) * tan30 / 2.0
    dtr = pi / 180.0
    
    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr
    
    y1 = -(t + rf*math.cos(theta1) )
    z1 = -rf * math.sin(theta1)
    
    y2 = (t + rf*math.cos(theta2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * math.sin(theta2)
    
    y3 = (t + rf*math.cos(theta3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * math.sin(theta3)
    
    dnm = (y2-y1)*x3 - (y3-y1)*x2
    
    w1 = y1*y1 + z1*z1
    w2 = x2*x2 + y2*y2 + z2*z2
    w3 = x3*x3 + y3*y3 + z3*z3
    
    # x = (a1*z + b1)/dnm
    a1 = (z2-z1)*(y3-y1) - (z3-z1)*(y2-y1)
    b1= -( (w2-w1)*(y3-y1) - (w3-w1)*(y2-y1) ) / 2.0
    
    # y = (a2*z + b2)/dnm
    a2 = -(z2-z1)*x3 + (z3-z1)*x2
    b2 = ( (w2-w1)*x3 - (w3-w1)*x2) / 2.0
    
    # a*z^2 + b*z + c = 0
    a = a1*a1 + a2*a2 + dnm*dnm
    b = 2.0 * (a1*b1 + a2*(b2 - y1*dnm) - z1*dnm*dnm)
    c = (b2 - y1*dnm)*(b2 - y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re)

    d = b*b - 4.0*a*c
    if d < 0.0:
        return [1,0,0,0] 
    
    z0 = -0.5*(b + math.sqrt(d)) / a
    x0 = (a1*z0 + b1) / dnm
    y0 = (a2*z0 + b2) / dnm

    return [0,x0,y0,z0]

# Inverse kinematics

def angle_yz(x0, y0, z0, theta=None):
    y1 = -0.5*0.57735*f # f/2 * tg 30
    y0 -= 0.5*0.57735*e # shift center to edge
    # z = a + b*y
    a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1) / (2.0*z0)
    b = (y1-y0) / z0

    d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf)
    if d<0:
        return [1,0] 

    yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1) 
    zj = a + b*yj
    theta = math.atan(-zj / (y1-yj)) * 180.0 / pi + (180.0 if yj>y1 else 0.0)
    
    return [0,theta] # return error, theta

def Inverse(x0, y0, z0):
    theta1 = 0
    theta2 = 0
    theta3 = 0
    status = angle_yz(x0,y0,z0)

    if status[0] == 0:
        theta1 = status[1]
        status = angle_yz(x0*cos120 + y0*sin120,
                                   y0*cos120-x0*sin120,
                                   z0,
                                   theta2)
    if status[0] == 0:
        theta2 = status[1]
        status = angle_yz(x0*cos120 - y0*sin120,
                                   y0*cos120 + x0*sin120,
                                   z0,
                                   theta3)
    theta3 = status[1]

    return [status[0],theta1,theta2,theta3]






#------------------------------------------------------------------------------------------------

#Start Work with Delta


#%%                    Point_pid
#E,d1,d2,d3 = Inverse(35,10,-50) #pend
#E,d1,d2,d3 = Inverse(-20,20,-73) #p1
#E,d1,d2,d3 = Inverse(-10,-20,-73) #p1
#E,d1,d2,d3 = Inverse(-0,-0,-73) #p1
E,d1,d2,d3 = Inverse(20,0,-60)
#first place
#d1=-70
#d2=-71
#d3=-57
#-----------home------------
#d1=0
#d2=0
#d3=0
#---------------------------
#E=0
#if d1<-26 or d2<-27 or d3<-26 :
#    E=1
#d=0
if not E:
    error1=[]
    error2=[]
    error3=[]
    
    v1=[]
    v2=[]
    v3=[]
    
    vo1=[]
    vo2=[]
    vo3=[]
    
    L=1
    l1=1
    l2=1
    l3=1
    
    kp = 0.5
    ki = 0.1
    kd = 0.0
    de_old1 = 0
    error_old1=0
    de_old2 = 0
    error_old2=0
    de_old3 = 0
    error_old3=0
    
    def pid(p_star, Id, de_old, error_old):
    
        pp = Pos_Actual_r(Id)
        L=1
        if pp == -333333:
            L = 0
        l=1
        J = 0.999
        error = p_star - pp
    #    if abs(error) >600 or abs(error)<200 :
    #        kp=0.02
    #    else:
    #        kp=0.05        
        DE = error - error_old
        de_new = J * de_old + (DE) * (1 - J)
        vel = kp * (error) + kd * (de_new - de_old) + ki * (error + error_old)
        de_old = de_new
        error_old = error
        if abs(error) < .2:
            vel = 0
            l = 0
        return vel, de_old, error_old, l , L
    Start = timeit.default_timer()
    try:
        while (l1 or l2 or l3 ) and L:
    #        start = timeit.default_timer()
            if l1:
                vel1, de_old1, error_old1, l1 , L = pid( d1 , 1, de_old1, error_old1)
                error1.append(error_old1) 
            if l2:
                vel2, de_old2, error_old2, l2 , L = pid( d2 , 2, de_old2, error_old2)
                error2.append(error_old2)                           
            if l3:
                vel3, de_old3, error_old3, l3 , L = pid( d3 , 3, de_old3, error_old3)
                error3.append(error_old3)
            
            Set_Speed_rpm(1,vel1)       
            vo1.append(vel1)
    #        v1.append(Velocity_Actual_r(1))
          
            Set_Speed_rpm(2,vel2)
            vo2.append(vel2)
    #        v2.append(Velocity_Actual_r(2))
            
            Set_Speed_rpm(3,vel3)
            vo3.append(vel3)
    #        v3.append(Velocity_Actual_r(3))
                  
    #        stop = timeit.default_timer()
    #        print('Time: ', stop - start) 
                     
    except KeyboardInterrupt:
        Motion_x(0)
        pass
    
    stop = timeit.default_timer()
    print('Time: ', stop - Start)
    Motion_x(0)
#%%
#------------------------------velocity_actual------------------------------------------------------
#Set_fbn(2)
#Set_kp_ki(130,5)
v1=[]
v2=[]
v3=[]
Motion_z(-3)
for i in range(50):
    v1.append(Velocity_Actual_r(1))
    v2.append(Velocity_Actual_r(2))
    v3.append(Velocity_Actual_r(3))

Motion_z(0)

#%%            path

#--------------------------------Don't touch-----------------------------------
P=[]
num=10
D1=[]
D2=[]
D3=[]

#p_old=[0,-0,0]
#p_new=[-100,100,50]

E,x,y,z=Forward(Pos_Actual_r(1), Pos_Actual_r(2), Pos_Actual_r(3))
if E == 1 :
    print("fault")
    time.sleep(10)
else :
    p_o=[x,y,z]
    p_n=[0,0,-65]
    
def pid_f(p_star, p_starold, Id, de_old, error_old,kp,ki,kd,i):
    pp = Pos_Actual_r(Id)
    L=1
    if pp == -333333:
        L = 0
    l=1
    J = 0.999
    error = p_star - pp      
    DE = error - error_old
    de_new = J * de_old + (DE) * (1 - J)

    if i ==0 :
        vel= abs(p_star - p_starold)* 1.5 * np.sign(error)
    elif i >= (num-1):
        kp=1
        vel = kp * (error) + kd * (de_new - de_old) + ki * (error + error_old)
    else:
        vel= abs(p_star - p_starold)* 2 * np.sign(error)
    de_old = de_new
    error_old = error
#    if abs(error) < (abs(p_star - p_starold) *1):
    if i < (num-1):
        if abs(error) < 1.5:
            i +=1
            l=0
#            if i > num-1 :
#                vel = 0
#                l = 0    
    else:
        if abs(error) < .1:
            vel = 0
            l = 0   
            
    return vel, de_old, error_old, l , L , i

for j in range(num):
    p0 = ((p_n[0] - p_o[0]) / num * (j+1) ) + p_o[0]
    p1 = ((p_n[1] - p_o[1]) / num * (j+1) ) + p_o[1]
    p2 = ((p_n[2] - p_o[2]) / num * (j+1) ) + p_o[2]
    P.append([p0,p1,p2])
for p in P:
    E,d1,d2,d3=Inverse(p[0],p[1],p[2])
    if d1<-25 or d2<-25 or d3<-25 :
        E=1
    else : 
        D2.append(d2)
        D3.append(d3)
        D1.append(d1)
error1=[]
error2=[]
error3=[]
v1=[]
v2=[]
v3=[]
L=1
l1=1
l2=1
l3=1
kp = .1
ki = 0
kd = 0
i1,i2,i3=0,0,0
k=0

de_old1 = 0
error_old1=0
de_old2 = 0
error_old2=0
de_old3 = 0
error_old3=0
if not E :
    try:
        while (l1 or l2 or l3 ) and L:
            #start = timeit.default_timer()
            if l1:
                vel1, de_old1, error_old1, l1 , L ,i1= pid_f( D1[k] , D1[k-1], 1, de_old1, error_old1,kp,ki,kd,i1)
                error1.append(error_old1)   
                Set_Speed_rpm(1,vel1 )  
                v1.append((vel1,i1))   
            if l2:
                vel2, de_old2, error_old2, l2 , L ,i2 = pid_f( D2[k] , D2[k-1], 2, de_old2, error_old2,kp,ki,kd,i2)
                error2.append(error_old2)
    #                Set_Speed_rpm(2,vel2 * np.abs(dd2 / mx))
                Set_Speed_rpm(2,vel2 )
                v2.append((vel2,i2))                           
            if l3:
                vel3, de_old3, error_old3, l3 , L ,i3= pid_f( D3[k] , D3[k-1], 3, de_old3, error_old3,kp,ki,kd,i3)
                error3.append(error_old3)
                Set_Speed_rpm(3,vel3 )
                v3.append((vel3,i3))
    
            if (i2==(k+1) and i1==(k+1) and i3==(k+1)) :
                k = k+1
                l1,l2,l3 = 1,1,1
            if k > (num-1):
                L=0
    except KeyboardInterrupt:
        Motion_x(0)
        pass
                
       # stop = timeit.default_timer()
       # print('Time: ', stop - start) 
    
    Motion_x(0)
    
#%%            path circle
P=[]
num_rounds=3
num=100 * num_rounds
D1=[]
D2=[]
D3=[]
D4=[]

def pid_f(p_star, p_starold, Id, de_old, error_old,kp,ki,kd,i):
    pp = Pos_Actual_r(Id)
    L=1
    if pp == -333333:
        L = 0
    l=1
    J = 0.999
    error = p_star - pp      
    DE = error - error_old
    de_new = J * de_old + (DE) * (1 - J)
    
    if i >= (num-1):
        kp=.5
        vel = kp * (error) + kd * (de_new - de_old) + ki * (error + error_old)
#    print(Velocity_Actual_r(Id),"---",Id)
    else:
        vel= abs(p_star - p_starold)* 4 * np.sign(error)
    de_old = de_new
    error_old = error
#    if abs(error) < (abs(p_star - p_starold) *1):
    if i < (num-1):
        if abs(error) < 1.5:
            i +=1
            l=0
#            if i > num-1 :
#                vel = 0
#                l = 0    
    else:
        if abs(error) < .5:
            vel = 0
            l = 0   
            
    return vel, de_old, error_old, l , L , i, pp
#--------------------------------------------
teta = np.linspace(0,num_rounds*2*np.pi,num=num)
r=10
for t in teta:
    p0 = r * np.cos(t)
    p1 = r * np.sin(t)
    p2 = -50
    P.append([p0,p1,p2])
for p in P:
    E,d1,d2,d3=Inverse(p[0],p[1],p[2])
    D2.append(d2)
    D3.append(d3)
    D1.append(d1)
error1=[]
error2=[]
error3=[]


v1=[]
v2=[]
v3=[]

v11=[]
v22=[]
v33=[]

PP1=[]
PP2=[]
PP3=[]

T1=[]
T2=[]
T3=[]

L=1
l1=1
l2=1
l3=1
kp = 2  #kp = 2  
ki = 0.5
kd = 0.2
i1,i2,i3=0,0,0
k=0

de_old1 = 0
error_old1=0
de_old2 = 0
error_old2=0
de_old3 = 0
error_old3=0

try:
    while (l1 or l2 or l3 ) and L:
        #start = timeit.default_timer()
        if l2:
            vel2, de_old2, error_old2, l2 , L ,i2, pp2 = pid_f( D2[k] , D2[k-1], 2, de_old2, error_old2,kp,ki,kd,i2)
            error2.append(error_old2)
#                Set_Speed_rpm(2,vel2 * np.abs(dd2 / mx))
            Set_Speed_rpm(2,vel2 )
            PP2.append(pp2)
#            v2.append((vel2,i2))
            v22.append(vel2)
#            T2.append(Torque_Actual(2))
        if l1:
            vel1, de_old1, error_old1, l1 , L , i1, pp1 = pid_f( D1[k] , D1[k-1], 1, de_old1, error_old1,kp,ki,kd,i1)
            error1.append(error_old1)   
            Set_Speed_rpm(1,vel1 )
            PP1.append(pp1)
#            v1.append((vel1,i1))
            v11.append(vel1)
#            T1.append(Torque_Actual(1))                               
        if l3:
            vel3, de_old3, error_old3, l3 , L , i3, pp3 = pid_f( D3[k] , D3[k-1], 3, de_old3, error_old3,kp,ki,kd,i3)
            error3.append(error_old3)
            Set_Speed_rpm(3,vel3 )
            PP3.append(pp3)
#            v3.append((vel3,i3))
            v33.append(vel3)
#            T3.append(Torque_Actual(3))
        if (i2==(k+1) and i1==(k+1) and i3==(k+1) ) :
            k = k+1
            l1,l2,l3 = 1,1,1
        if k > (num-1):
            L=0
except KeyboardInterrupt:
    Motion_x(0)
    pass
            
   # stop = timeit.default_timer()
   # print('Time: ', stop - start) 

Motion_x(0)
    
#plot

#position = []
#for i in range(len(PP3)):
#    e,x,y,z = Forward(PP1[i],PP2[i],PP3[i])
#    position.append([x,y,z])
#position = np.array(position)
#plt.plot(position[:,0], position[:,1])
#plt.axis('equal')
#%% plot x-y

position = []
for i in range(len(Pp1)):
    e,x,y,z = Forward(Pp1[i],Pp2[i],Pp3[i])
    position.append([x,y,z])
position = np.array(position)
plt.plot(position[:,0], position[:,1])
plt.axis('equal')
#%%                    Point_pid_pick_and_place
#E,d1,d2,d3 = Inverse(35,10,-50) #pend
#E,d1,d2,d3 = Inverse(-20,20,-73) #p1
#E,d1,d2,d3 = Inverse(-10,-20,-73) #p1
#E,d1,d2,d3 = Inverse(-0,-0,-73) #p1
#d1=20.71
#d2=-11.61
#d3=47.53
def pick(x,y,z):
    E,d1,d2,d3 = Inverse(x,y,z)
    if d1<-26 or d2<-27 or d3<-26 :
        E=1
    #d=0
    if not E:
        error1=[]
        error2=[]
        error3=[]
        
        v1=[]
        v2=[]
        v3=[]
        
        vo1=[]
        vo2=[]
        vo3=[]
        
        L=1
        l1=1
        l2=1
        l3=1
        
        kp = .2
        ki = 0.05
        kd = 0.0
        de_old1 = 0
        error_old1=0
        de_old2 = 0
        error_old2=0
        de_old3 = 0
        error_old3=0
        
        def pid(p_star, Id, de_old, error_old):
        
            pp = Pos_Actual_r(Id)
            L=1
            if pp == -333333:
                L = 0
            l=1
            J = 0.999
            error = p_star - pp
        #    if abs(error) >600 or abs(error)<200 :
        #        kp=0.02
        #    else:
        #        kp=0.05        
            DE = error - error_old
            de_new = J * de_old + (DE) * (1 - J)
            vel = kp * (error) + kd * (de_new - de_old) + ki * (error + error_old)
            de_old = de_new
            error_old = error
            if abs(error) < 3:
                vel = 0
                l = 0
            return vel, de_old, error_old, l , L
        Start = timeit.default_timer()
        try:
            while (l1 or l2 or l3 ) and L:
        #        start = timeit.default_timer()
                if l1:
                    vel1, de_old1, error_old1, l1 , L = pid( d1 , 1, de_old1, error_old1)
                    error1.append(error_old1) 
                if l2:
                    vel2, de_old2, error_old2, l2 , L = pid( d2 , 2, de_old2, error_old2)
                    error2.append(error_old2)                           
                if l3:
                    vel3, de_old3, error_old3, l3 , L = pid( d3 , 3, de_old3, error_old3)
                    error3.append(error_old3)
                
                Set_Speed_rpm(1,vel1)       
                vo1.append(vel1)
        #        v1.append(Velocity_Actual_r(1))
              
                Set_Speed_rpm(2,vel2)
                vo2.append(vel2)
        #        v2.append(Velocity_Actual_r(2))
                
                Set_Speed_rpm(3,vel3)
                vo3.append(vel3)
        #        v3.append(Velocity_Actual_r(3))
                      
        #        stop = timeit.default_timer()
        #        print('Time: ', stop - start) 
                         
        except KeyboardInterrupt:
            Motion_x(0)
            pass
        
        stop = timeit.default_timer()
        print('Time: ', stop - Start)
        Motion_x(0)    

#Suction(1)    
pick(0,0,-73)
pick(35,10,-50)
#Suction(0)  
time.sleep(.1) 
#Suction(1)    
pick(-20,20,-73)
pick(35,10,-50)
#Suction(0)   
time.sleep(.1) 
#Suction(1)
pick(-10,-20,-73)
pick(35,10,-50)
#Suction(0) 
    
#%%
#E,d1,d2,d3=Inverse(35,10,-50)
#E,d1,d2,d3 = Inverse(-20,20,-73)
#E,d1,d2,d3=Inverse(0,0,-70)
def Guassian(x,y,z):
    E,d1,d2,d3 = Inverse(x,y,z)
    if d1<-26 or d2<-29 or d3<-26 :
        E=1
    #d=0
    if not E:
        p1 = Pos_Actual_r(1)
        p2 = Pos_Actual_r(2)
        p3 = Pos_Actual_r(3)
        
        h1 = (d1-p1)/2 +p1
        h2 = (d2-p2)/2 +p2
        h3 = (d3-p3)/2 +p3 
        
        max=np.max([abs(d1-p1),abs(d2-p2),abs(d3-p3)])
        S=40
        s1 = S * abs(d1-p1 )/ max
        s2 = S * abs(d2-p2 )/ max
        s3 = S * abs(d3-p3 )/ max
        
        sigma1= abs(d1-p1)/5
        sigma2= abs(d2-p2)/5
        sigma3= abs(d3-p3)/5
        
        error1=[]
        error2=[]
        error3=[]
        
        v1=[]
        v2=[]
        v3=[]
        
        vo1=[]
        vo2=[]
        vo3=[]
        
        L=1
        l1=1
        l2=1
        l3=1
        
        
        
        def guassian(p_star, Id,h,S,sigma):
        #    time.sleep(.008)
            pp = Pos_Actual_r(Id)
            L=1
            if pp == -333333:
                L = 0
            l=1    
            error = p_star - pp
            
            errorp = h - pp
            vel = S* (math.e ** (-1*abs(errorp)/sigma)) * np.sign(error)
            print(vel)
        
            if abs(error) < .5:
                vel = 0
                l = 0
            return vel,error, l , L
        start = timeit.default_timer()
        try:
            while (l1 or l2 or l3 ) and L:
        #        start = timeit.default_timer()
                if l1:
                    vel1,error_old1, l1 , L = guassian( d1 , 1,h1,s1,sigma1)
                    error1.append(error_old1)   
                    Set_Speed_rpm(1,vel1)    
                    vo1.append(vel1)
        #            v1.append(Velocity_Actual_r(1)) 
                if l2:
                    vel2,error_old2, l2 , L = guassian( d2  , 2,h2,s2,sigma2)
                    error2.append(error_old2)
                    Set_Speed_rpm(2,vel2)
                    vo2.append(vel2)
        #            v2.append(Velocity_Actual_r(2))                            
                if l3:
                    vel3,error_old3, l3 , L = guassian( d3 , 3,h3,s3,sigma3)
                    error3.append(error_old3)
                    Set_Speed_rpm(3,vel3)
        #            v3.append(Velocity_Actual_r(3))
                    vo3.append(vel3)
                   
        #        stop = timeit.default_timer()
        #        print('Time: ', stop - start) 
        except KeyboardInterrupt:
            Motion_x(0)
            pass
        stop = timeit.default_timer()
        print('Time: ', stop - start) 
        Motion_x(0)
        
Start = timeit.default_timer()        
#Suction(1)    
##Guassian(0,0,-73)
#Guassian(35,10,-50)
##Suction(0)  
##time.sleep(.1) 
##Suction(1)    
##Guassian(-20,20,-73)
##Guassian(35,10,-50)
##Suction(0)   
##time.sleep(.1) 
##Suction(1)
##Guassian(-10,-20,-73)
##Guassian(35,10,-50)
##Suction(0) 
Guassian(-20,-10,-61)
Guassian(-20,-10,-72)
#time.sleep(.5) 
Guassian(-20,-10,-61)

Guassian(40,15,-43)
#Guassian(35,10,-50)
#Suction(0)  
time.sleep(.1) 
#Suction(1)    
Guassian(0,0,-65)
Guassian(0,0,-76) 
Guassian(0,0,-65)
Guassian(40,15,-43)
#Guassian(35,10,-50)
#Suction(0)   
time.sleep(.1) 
#Suction(1)
Guassian(-10,-20,-62.5)
Guassian(-10,-20,-73.5)
Guassian(-10,-20,-62.5)
#time.sleep(.5) 
Guassian(40,15,-43)
#Guassian(35,10,-50)
#Suction(0) 

time.sleep(.1) 
#Suction(1)
Guassian(-20,20,-63)
Guassian(-20,20,-74)
Guassian(-20,20,-63)
#time.sleep(.5) 
Guassian(40,15,-43)
#Guassian(35,10,-50)
#Suction(0) 


Sstop = timeit.default_timer()
print('Time:---- ', Sstop - Start) 


#%%            path
P=[]
num=20
D1=[]
D2=[]
D3=[]
Start = timeit.default_timer()
#p_old=[0,-0,0]
#p_new=[-100,100,50]
E,x,y,z=Forward(Pos_Actual_r(1), Pos_Actual_r(2), Pos_Actual_r(3))
if E == 1 :
    print("fault")
    time.sleep(10)
else :

    p_o=[x,y,z]
    p_n=[-10,10,-65]
#    p_n=[35,10,-50]
    
def pid_f(p_star, p_starold, Id, de_old, error_old,kp,ki,kd,i,s,sigma):
    pp = Pos_Actual_r(Id)
    L=1
    if pp == -333333:
        L = 0
    l=1
    J = 0.999
    error = p_star - pp      
    DE = error - error_old
    de_new = J * de_old + (DE) * (1 - J)
#    s= 1
    if i ==0 :
        vel = s* (math.e ** (-1*abs(error)/sigma)) * np.sign(error)
    elif i >= (num-1):
        kp=1
        vel = kp * (error) + kd * (de_new - de_old) + ki * (error + error_old)
    else:
        vel= abs(p_star - p_starold)* 3 * np.sign(error)
    de_old = de_new
    error_old = error
#    if abs(error) < (abs(p_star - p_starold) *1):
    if i < (num-1):
        if abs(error) < 2.5:
            i +=1
            l=0
#            if i > num-1 :
#                vel = 0
#                l = 0    
    else:
        if abs(error) < .1:
            vel = 0
            l = 0   
            
    return vel, de_old, error_old, l , L , i

for j in range(num):
    p0 = ((p_n[0] - p_o[0]) / num * (j+1) ) + p_o[0]
    p1 = ((p_n[1] - p_o[1]) / num * (j+1) ) + p_o[1]
    p2 = ((p_n[2] - p_o[2]) / num * (j+1) ) + p_o[2]
    P.append([p0,p1,p2])
del P[-5:-1]
num = len(P)
for p in P:
    E,d1,d2,d3=Inverse(p[0],p[1],p[2])
    if d1<-25 or d2<-25 or d3<-25 :
        E=1
    else : 
        D2.append(d2)
        D3.append(d3)
        D1.append(d1)
p1 = Pos_Actual_r(1)
p2 = Pos_Actual_r(2)
p3 = Pos_Actual_r(3)

max=np.max([abs(D1[0]-p1),abs(D2[0]-p2),abs(D3[0]-p3)])
S=40
s1 = S * abs(D1[0]-p1 )/ max
s2 = S * abs(D2[0]-p2 )/ max
s3 = S * abs(D3[0]-p3 )/ max

sigma1= abs(D1[0]-p1)/3
sigma2= abs(D2[0]-p2)/3
sigma3= abs(D3[0]-p3)/3
error1=[]
error2=[]
error3=[]
v1=[]
v2=[]
v3=[]
L=1
l1=1
l2=1
l3=1
kp = .1
ki = 0
kd = 0
i1,i2,i3=0,0,0
k=0

de_old1 = 0
error_old1=0
de_old2 = 0
error_old2=0
de_old3 = 0
error_old3=0
if not E :
    try:
        while (l1 or l2 or l3 ) and L:
            #start = timeit.default_timer()
            if l1:
                vel1, de_old1, error_old1, l1 , L ,i1= pid_f( D1[k] , D1[k-1], 1, de_old1, error_old1,kp,ki,kd,i1,s1,sigma1)
                error1.append(error_old1)   
                Set_Speed_rpm(1,vel1 )  
                v1.append((vel1,i1))   
            if l2:
                vel2, de_old2, error_old2, l2 , L ,i2 = pid_f( D2[k] , D2[k-1], 2, de_old2, error_old2,kp,ki,kd,i2,s2,sigma2)
                error2.append(error_old2)
    #                Set_Speed_rpm(2,vel2 * np.abs(dd2 / mx))
                Set_Speed_rpm(2,vel2 )
                v2.append((vel2,i2))                           
            if l3:
                vel3, de_old3, error_old3, l3 , L ,i3= pid_f( D3[k] , D3[k-1], 3, de_old3, error_old3,kp,ki,kd,i3,s3,sigma3)
                error3.append(error_old3)
                Set_Speed_rpm(3,vel3 )
                v3.append((vel3,i3))
    
            if (i2==(k+1) and i1==(k+1) and i3==(k+1)) :
                k = k+1
                l1,l2,l3 = 1,1,1
            if k > (num-1):
                L=0
    except KeyboardInterrupt:
        Motion_x(0)
        pass
                
       # stop = timeit.default_timer()
       # print('Time: ', stop - start) 
    
    Motion_x(0)

Sstop = timeit.default_timer()
print('Time:---- ', Sstop - Start)     

#%%
start = timeit.default_timer()
v=[]
Motion_z(-10)
while 1:
    stop = timeit.default_timer()
    v.append(Velocity_Actual_r(1))
    if stop - start > .4:
        Motion_z(0)
        break
Motion_z(0)


#%%
import matplotlib.pyplot as plt
plt.figure(1)
#plt.plot(vo1)
#plt.hold(True)
plt.plot(v11)

plt.figure(2)
#plt.plot(vo2)
#plt.hold(True)
plt.plot(v22)

plt.figure(3)
#plt.plot(vo3)
#plt.hold(True)
plt.plot(v33)


plt.figure(4)
#plt.plot(vo1)
#plt.hold(True)
plt.plot(PP1)

plt.figure(5)
#plt.plot(vo2)
#plt.hold(True)
plt.plot(PP2)

plt.figure(6)
#plt.plot(vo3)
#plt.hold(True)
plt.plot(PP3)


#plt.figure(7)
##plt.plot(vo1)
##plt.hold(True)
#plt.plot(T1)
#
#plt.figure(8)
##plt.plot(vo2)
##plt.hold(True)
#plt.plot(T2)
#
#plt.figure(9)
##plt.plot(vo3)
##plt.hold(True)
#plt.plot(T3)


#----------------------------------------PID_FARAZ-------------------------------------
#%%  
#num=100 * 5
#D1=[]
#D2=[]
#D3=[]
#teta = np.linspace(0,5*2*np.pi,num=num)
#r=30
#for t in teta:
#    p0 = r * np.cos(t)
#    p1 = r * np.sin(t)
#    p2 = -50
#    P.append([p0,p1,p2])
#for p in P:
#    E,d1,d2,d3=Inverse(p[0],p[1],p[2])
#    D2.append(d2)
#    D3.append(d3)
#    D1.append(d1)


Pp1=[]
Pp2=[]
Pp3=[]

Pd1=[]
Pd2=[]
Pd3=[]

VEL1 = []
VEL2 = []
VEL3 = []
#-----------------point------------
E,d1,d2,d3 = Inverse(30,0,-50)
#d1,d2,d3 = 0,0,0
D1=[d1]
D2=[d2]
D3=[d3]
Pd1.append(d1)
Pd2.append(d2)
Pd3.append(d3)
#----------------------------------

pp11,pp22,pp33 = [],[],[]
vvel1,vvel2,vvel3 =[],[],[]
eerror=[]
k = 0
ii = 0
ii1,ii2,ii3 = 0,0,0
iio1,iio2,iio3 = 0,0,0
iin1,iin2,iin3 = 0,0,0
l1,l2,l3 = 1,1,1
#------------circle--------------
#kp = 2   #1.2
#kd = 1    #0.6
#ki = 0.1    #0.1
#-----------regulation----------------
kp = 0.5
kd = 0.1
ki = 0.05
#-------------------------------
error_old1,error_old2,error_old3 = 0,0,0
de_old1,de_old2,de_old3 = 0,0,0
a=[]
def pid(p_star, Id, de_old, error_old,ii):
    l=1
    J = 0.999
    if Id == 1:
        pp = Pos_Actual_r(Id)
        pp11.append(pp)
    elif Id == 2:
        pp = Pos_Actual_r(Id)
        pp22.append(pp)
    elif Id == 3:
        pp = Pos_Actual_r(Id)
        pp33.append(pp)

        
#    pp = Pos_Actual_r(Id)
#    pp = Pos_Actual_r(Id)
    error = p_star - pp
    eerror.append(error)
    DE = error - error_old
    de_new = J * de_old + (DE) * (1 - J)
#    if ii == 0:
#        kp = 0.1       #0.1
#        kd = 0.01     #0.01
#        ki = 0.001   #0.001
#        vel = kp_1 * (error) + kd_1 * (de_new - de_old) + ki_1 * (error + error_old)
#    else:
#        vel = kp * (error) + kd * (de_new - de_old) + ki * (error + error_old)
    vel = kp * (error) + kd * (de_new - de_old) + ki * (error + error_old)
    de_old = de_new
    error_old = error
    
    if abs(error) < 2:
        ii = ii + 1
        
    if ii == len(D2):
#    if ii == 1:
        l = 0
    return vel, pp, de_old, error_old, l , ii

start_time = time.time()
try:   
    while l1 or l2 or l3:
        if l1:
            vel1,pp1, de_old1, error_old1, l1, iin1 = pid(D1[iin1], 1, de_old1, error_old1,iin1)
#            vvel1.append(vel1)
            Pp1.append(pp1)
            VEL1.append(vel1)
        if l2:
            vel2, pp2, de_old2, error_old2, l2, iin2 = pid(D2[iin2], 2, de_old2, error_old2,iin2)
#            vvel2.append(vel2)
            Pp2.append(pp2)
            VEL2.append(vel2)
        if l3:
            vel3, pp3, de_old3, error_old3, l3, iin3 = pid(D3[iin3], 3, de_old3, error_old3,iin3)
#            vvel3.append(vel3)
            Pp3.append(pp3)
            VEL3.append(vel3)



#        if l1:
#            vel1, de_old1, error_old1, l1, iin1 = pid(0, 1, de_old1, error_old1,iin1)
#        if l2:
#            vel2, de_old2, error_old2, l2, iin2 = pid(0, 2, de_old2, error_old2,iin2)
#        if l3:
#            vel3, de_old3, error_old3, l3, iin3 = pid(0, 3, de_old3, error_old3,iin3)


    #    if iio1 != iin1 and iio2 != iin2 and iio3 != iin3:
    #        ii1 = iin1 
    #        ii2 = iin2
    #        ii3 = iin3
  
    #        iio1 = iin1
    #        iio2 = iin2
    #        iio3 = iin3

        Set_Speed_rpm(1,vel1)
        Set_Speed_rpm(2,vel2)
        Set_Speed_rpm(3,vel3)

#    print(vel1,vel2,vel3)
except KeyboardInterrupt:
    Motion_x(0)
    pass
            
   # stop = timeit.default_timer()
   # print('Time: ', stop - start) 
print("--- %s seconds ---" % (time.time() - start_time))
Motion_x(0)

plt.figure(1)
plt.plot(Pp1)
plt.hold
plt.plot(Pd1)
plt.figure(2)
plt.plot(Pp2)
plt.hold
plt.plot(Pd2)
plt.figure(3)
plt.plot(Pp3)
plt.hold
plt.plot(Pd3)

plt.figure(4)
plt.plot(VEL1)
plt.hold
plt.plot(VEL2)
plt.hold
plt.plot(VEL3)

#scipy.io.savemat('E:\Delta_4_DoF\pid_helix\pp1pid.mat', mdict={'pp1pid': pp11})
#scipy.io.savemat('E:\Delta_4_DoF\pid_helix\pp2pid.mat', mdict={'pp2pid': pp22})
#scipy.io.savemat('E:\Delta_4_DoF\pid_helix\pp3pid.mat', mdict={'pp3pid': pp33})


#scipy.io.savemat('E:\Delta_4_DoF\pid_helix\Vel1pid.mat', mdict={'Vel1pid': vvel1})
#scipy.io.savemat('E:\Delta_4_DoF\pid_helix\Vel2pid.mat', mdict={'Vel2pid': vvel2})
#scipy.io.savemat('E:\Delta_4_DoF\pid_helix\Vel3pid.mat', mdict={'Vel3pid': vvel3})


#with open("E:\Delta_4_DoF\pid_helix\pp1.txt", "wb") as fb:
#    pickle.dump(pp11,fb)
#with open("E:\Delta_4_DoF\pid_helix\pp2.txt", "wb") as fb:
#    pickle.dump(pp22,fb)
#with open("E:\Delta_4_DoF\pid_helix\pp3.txt", "wb") as fb:
#    pickle.dump(pp33,fb)


#%%      ----------------------PID_cicle_Time--------------------------
#Set_fbn(0)
#Set_kp_ki(300,2)
t = 0
r = 10
#q = 6
q = 12*6                   #2x num of circles
#kp=.9
#ki=0.04
kp = 0.6
ki = 0.05
kd = 0.02
i1,i2,i3 = 0,0,0
de_old1 = 0
error_old1 = 0
de_old2 = 0
error_old2 = 0
de_old3 = 0
error_old3 = 0


Pp1=[]
Pp2=[]
Pp3=[]

Pd1=[]
Pd2=[]
Pd3=[]

VEL1 = []
VEL2 = []
VEL3 = []

pp1 = Pos_Actual_r(1)
pp2 = Pos_Actual_r(2)
pp3 = Pos_Actual_r(3)

def pid_f(p_star, Id, de_old, error_old,kp,ki,kd,i):
    pp = Pos_Actual_r(Id)
    L=1
    if pp == -333333:
        L = 0
    l=1
    J = 0.999
    error = p_star - pp      
    DE = error - error_old
    de_new = J * de_old + (DE) * (1 - J)
    
    vel = kp * (error) + kd * (de_new - de_old) + ki * (error_old)

    de_old = de_new 
    error_old = error + error_old
#    if abs(error) < (abs(p_star - p_starold) *1):
            
    return vel, pp, de_old, error_old, l , L , i

#T= 8
T= 24*6                #time of processs
start = timeit.default_timer()
try:
#    while t<(1.4*T):
    sinaaa = 0
    for sinaaa in range(2):
        while t<(1.4*T):
            Tt = t/T
            t = timeit.default_timer() - start
            if t<T:
                px = r * np.cos(q*np.pi*Tt) 
                py = r * np.sin(q*np.pi*Tt)
                pz=-60
            else:
                px = r * np.cos(q*np.pi*0) 
                py = r * np.sin(q*np.pi*0)
                pz=-60
    #            kp=.3
    #            ki=0.02
    #        px= (100-0)*t/T + 0
    #        px = 100*Tt
    #        py = 0
    #        pz = 100*Tt + 0
            E,d1,d2,d3=Inverse(px,py,pz)
            Pd1.append(d1)
            Pd2.append(d2)
            Pd3.append(d3)
    
    #        d1=(d1-pp1)*t/T + pp1
    #        d2=(d2-pp2)*t/T + pp2
    #        d3=(d3-pp3)*t/T + pp3
    
            
            if not E:
                vel1, pp1, de_old1, error_old1, l1 , L ,i1= pid_f(d1, 1, de_old1, error_old1, kp, ki, kd, i1)
                Pp1.append(pp1)
                VEL1.append(vel1)
                vel2, pp2, de_old2, error_old2, l2 , L ,i2= pid_f(d2, 2, de_old2, error_old2, kp, ki, kd, i2)
                Pp2.append(pp2)
                VEL2.append(vel2)
                vel3, pp3, de_old3, error_old3, l3 , L ,i3= pid_f(d3, 3, de_old3, error_old3, kp, ki, kd, i3)
                Pp3.append(pp3)
                VEL3.append(vel3)
    
                
                Set_Speed_rpm(1,vel1)
                Set_Speed_rpm(2,vel2)
                Set_Speed_rpm(3,vel3)
    
    
except KeyboardInterrupt:
    Motion_x(0)
    pass    
Motion_x(0)    
plt.figure(1)
plt.plot(Pp1)
plt.hold
plt.plot(Pd1)
plt.figure(2)
plt.plot(Pp2)
plt.hold
plt.plot(Pd2)
plt.figure(3)
plt.plot(Pp3)
plt.hold
plt.plot(Pd3)

plt.figure(4)
plt.plot(VEL1)
plt.hold
plt.plot(VEL2)
plt.hold
plt.plot(VEL3)

#%%      ----------------------Sliding_cicle_Time_Arman--------------------------
#Set_fbn(0)
#Set_kp_ki(300,2)
t = 0
r = 20
q = 6
#kp=.9
#ki=0.04
kp = 0.6
ki = 0.05
kd = 0.02
i1,i2,i3 = 0,0,0
de_old1 = 0
error_old1 = 0
de_old2 = 0
error_old2 = 0
de_old3 = 0
error_old3 = 0


Pp1=[]
Pp2=[]
Pp3=[]

Pd1=[]
Pd2=[]
Pd3=[]

VEL1 = []
VEL2 = []
VEL3 = []

pp1 = Pos_Actual_r(1)
pp2 = Pos_Actual_r(2)
pp3 = Pos_Actual_r(3)

def pid_f(p_star, Id, de_old, error_old,kp,ki,kd,i):
    pp = Pos_Actual_r(Id)
    L=1
    if pp == -333333:
        L = 0
    l=1
    J = 0.999
    error = p_star - pp      
    DE = error - error_old
    de_new = J * de_old + (DE) * (1 - J)
    S = error      # + 0.005*(de_new)
    delta_star = 0.3 
    vel = kp * (error) + kd * (de_new - de_old) + ki * (error_old) - delta_star*(math.tanh(2*S))

    de_old = de_new 
    error_old = error + error_old
#    if abs(error) < (abs(p_star - p_starold) *1):
            
    return vel, pp, de_old, error_old, l , L , i
#%%
T= 10
start = timeit.default_timer()
try:
    while t<(1.4*T):
        Tt = t/T
        t = timeit.default_timer() - start
        if t<T:
            px = r * np.cos(q*np.pi*Tt) 
            py = r * np.sin(q*np.pi*Tt)
            pz=-60
        else:
            px = r * np.cos(q*np.pi*0) 
            py = r * np.sin(q*np.pi*0)
            pz=-60
#            kp=.3
#            ki=0.02
#        px= (100-0)*t/T + 0
#        px = 100*Tt
#        py = 0
#        pz = 100*Tt + 0
        E,d1,d2,d3=Inverse(px,py,pz)
        Pd1.append(d1)
        Pd2.append(d2)
        Pd3.append(d3)

#        d1=(d1-pp1)*t/T + pp1
#        d2=(d2-pp2)*t/T + pp2
#        d3=(d3-pp3)*t/T + pp3

        
        if not E:
            vel1, pp1, de_old1, error_old1, l1 , L ,i1= pid_f(d1, 1, de_old1, error_old1, kp, ki, kd, i1)
            Pp1.append(pp1)
            VEL1.append(vel1)
            vel2, pp2, de_old2, error_old2, l2 , L ,i2= pid_f(d2, 2, de_old2, error_old2, kp, ki, kd, i2)
            Pp2.append(pp2)
            VEL2.append(vel2)
            vel3, pp3, de_old3, error_old3, l3 , L ,i3= pid_f(d3, 3, de_old3, error_old3, kp, ki, kd, i3)
            Pp3.append(pp3)
            VEL3.append(vel3)

            
            Set_Speed_rpm(1,vel1)
            Set_Speed_rpm(2,vel2)
            Set_Speed_rpm(3,vel3)

            
        
except KeyboardInterrupt:
    Motion_x(0)
    pass    
Motion_x(0)    
plt.figure(1)
plt.plot(Pp1)
plt.hold
plt.plot(Pd1)
plt.figure(2)
plt.plot(Pp2)
plt.hold
plt.plot(Pd2)
plt.figure(3)
plt.plot(Pp3)
plt.hold
plt.plot(Pd3)

plt.figure(4)
plt.plot(VEL1)
plt.hold
plt.plot(VEL2)
plt.hold
plt.plot(VEL3)
 
#%%
#----------------------------------Sliding_FARAZ--------------------------------
D1=[]
D2=[]
D3=[]
num=100 * 2
teta = np.linspace(0,5*2*np.pi,num=num)
r=30
for t in teta:
    p0 = r * np.cos(t)
    p1 = r * np.sin(t)
    p2 = -50
    P.append([p0,p1,p2])
for p in P:
    E,d1,d2,d3=Inverse(p[0],p[1],p[2])
    D1.append(d1)
    D2.append(d2)
    D3.append(d3)


#-----------------point------------
#E,d1,d2,d3 = Inverse(30,0,-50)
#d1,d2,d3 = 0,0,0
#D1=[d1]
#D2=[d2]
#D3=[d3]
#----------------------------------
k = 0
ii = 0
ii1,ii2,ii3 = 0,0,0
iio1,iio2,iio3 = 0,0,0
iin1,iin2,iin3 = 0,0,0
l1,l2,l3 = 1,1,1
kp = 0.6     #1.2
kd = 0.2     #0.6
ki = 0.3     #0.1
pp11,pp22,pp33 = [],[],[]
vvel1,vvel2,vvel3 = [],[],[]
eerror=[]
#k1 = 1
#k2 = 0.6
#delta_star = 1.4
V1 = []

error_old1,error_old2,error_old3 = 0,0,0
de_old1,de_old2,de_old3 = 0,0,0
a=[]
def pid(p_star, Id, de_old, error_old,ii):
    l=1
    J = 0.99
    L = 1
    if Id == 1:
        pp = Pos_Actual_r(Id)
        pp11.append(pp)
    elif Id == 2:
        pp = Pos_Actual_r(Id)

        pp22.append(pp)
    elif Id == 3:
        pp = Pos_Actual_r(Id)

        pp33.append(pp)


    error =  pp - p_star
    eerror.append(error)
    DE = error - error_old
    de_new = J * de_old + (DE) * (1 - J)
#    if ii == 0:
#        k1 = 1.8
#        k2 = 1.8
#        delta_star = 0.5
#        S = k1*(error)+k2*(de_new)
##        V = 1/2*(S**2)
##        V1.append(V)
#        vel = (-k1/k2)*(de_new) - delta_star*(math.tanh(S))
#    else:
    k1 = 1.6
    k2 = 1.8
    delta_star = 1.2

    S = k1*(error)+k2*(de_new)
#        V = 1/2*(S**2)
#        V1.append(V)
#        print(V)
    vel = (-k1/k2)*(de_new) - delta_star*(math.tanh(S))
    de_old = de_new
    error_old = error
#    if abs(error) < 3:
#        ii = ii + 1
#    if ii == len(D1):
#        l = 0

    if ii < (num-1):
        if abs(error) < 1.5:
            ii +=1
            l=0
#            if i > num-1 :
#                vel = 0
#                l = 0    
    else:
        if abs(error) < .5:
            vel = 0
            l = 0 
    return vel, de_old, error_old, l , ii
try:    
    while (l1 or l2 or l3) and L :
        if l1:
            vel1, de_old1, error_old1, l1, iin1 = pid(D1[k], 1, de_old1, error_old1,iin1)
            vvel1.append(vel1)
        if l2:
            vel2, de_old2, error_old2, l2, iin2 = pid(D2[k], 2, de_old2, error_old2,iin2)
            vvel2.append(vel2)
        if l3:
            vel3, de_old3, error_old3, l3, iin3 = pid(D3[k], 3, de_old3, error_old3,iin3)
            vvel3.append(vel3)

        
        Set_Speed_rpm(1,vel1)
        Set_Speed_rpm(2,vel2)
        Set_Speed_rpm(3,vel3)



        if (iin1==(k+1) and iin2==(k+1) and iin3==(k+1) ) :
            k = k+1
            l1,l2,l3 = 1,1,1
        if k > (num-1):
            L=0
except KeyboardInterrupt:
    Motion_x(0)
    pass
            
   # stop = timeit.default_timer()
   # print('Time: ', stop - start) 
print("--- %s seconds ---" % (time.time() - start_time))
Motion_x(0)

#scipy.io.savemat('E:\Delta_4_DoF\smc_circle\pp1SMC.mat', mdict={'pp1SMC': pp11})
#scipy.io.savemat('E:\Delta_4_DoF\smc_circle\pp2SMC.mat', mdict={'pp2SMC': pp22})
#scipy.io.savemat('E:\Delta_4_DoF\smc_circle\pp3SMC.mat', mdict={'pp3SMC': pp33})

#scipy.io.savemat('E:\Delta_4_DoF\smc_circle\Vel1SMC.mat', mdict={'Vel1SMC': vvel1})
#scipy.io.savemat('E:\Delta_4_DoF\smc_circle\Vel2SMC.mat', mdict={'Vel2SMC': vvel2})
#scipy.io.savemat('E:\Delta_4_DoF\smc_circle\Vel3SMC.mat', mdict={'Vel3SMC': vvel3})

#with open("E:\Delta_4_DoF\smc_circle\pp1.txt", "wb") as fb:
#    pickle.dump(pp11,fb)
#with open("E:\Delta_4_DoF\smc_circle\pp2.txt", "wb") as fb:
#    pickle.dump(pp22,fb)
#with open("E:\Delta_4_DoF\smc_circle\pp3.txt", "wb") as fb:
#    pickle.dump(pp33,fb)


#%%      ----------------------SMC_cicle_Time--------------------------
#Set_fbn(0)
#Set_kp_ki(300,2)
t = 0
r = 20
q = 1

i1,i2,i3 = 0,0,0
de_old1 = 0
error_old1 = 0
de_old2 = 0
error_old2 = 0
de_old3 = 0
error_old3 = 0


Pp1=[]
Pp2=[]
Pp3=[]

Pd1=[]
Pd2=[]
Pd3=[]

VEL1 = []
VEL2 = []
VEL3 = []

pp1 = Pos_Actual_r(1)
pp2 = Pos_Actual_r(2)
pp3 = Pos_Actual_r(3)

def pid_f(p_star, Id, de_old, error_old,i):
    pp = Pos_Actual_r(Id)
    L=1
    if pp == -333333:
        L = 0
    l=1
    J = 0.999
    error = p_star - pp      
    DE = error - error_old
    de_new = J * de_old + (DE) * (1 - J)
    k1 = 1    #1.6
    k2 = 4    #1.8
    delta_star = 0.03   #1.2
    kp = 0.04
    ki = 0.02
    S = k1*(error)   #+k2*(de_new)
    
    vel =  ki * (error_old) + kp * (error) - delta_star*(math.tanh(S))   # vel = (-k1/k2)*(de_new-de_old) - delta_star*(math.tanh(S))  (-k1/k2)*(de_new)
    de_old = de_new 
    error_old = error + error_old

    return vel, pp, de_old, error_old, l, L, i

T= 25
start = timeit.default_timer()
try:
    while t<(1.4*T):
        Tt = t/T
        t = timeit.default_timer() - start
        if t<T:
            px = r * np.cos(q*np.pi*Tt) 
            py = r * np.sin(q*np.pi*Tt)
            pz=-60
        else:
            px = r * np.cos(q*np.pi*0) 
            py = r * np.sin(q*np.pi*0)
            pz=-60
#            kp=.3
#            ki=0.02
#        px= (100-0)*t/T + 0
#        px = 100*Tt
#        py = 0
#        pz = 100*Tt + 0
        E,d1,d2,d3=Inverse(px,py,pz)
        Pd1.append(d1)
        Pd2.append(d2)
        Pd3.append(d3)

#        d1=(d1-pp1)*t/T + pp1
#        d2=(d2-pp2)*t/T + pp2
#        d3=(d3-pp3)*t/T + pp3

        
        if not E:
            vel1, pp1, de_old1, error_old1, l1 , L ,i1= pid_f(d1, 1, de_old1, error_old1, i1)
            Pp1.append(pp1)
            VEL1.append(vel1)
            vel2, pp2, de_old2, error_old2, l2 , L ,i2= pid_f(d2, 2, de_old2, error_old2, i2)
            Pp2.append(pp2)
            VEL2.append(vel2)
            vel3, pp3, de_old3, error_old3, l3 , L ,i3= pid_f(d3, 3, de_old3, error_old3, i3)
            Pp3.append(pp3)
            VEL3.append(vel3)
            
            Set_Speed_rpm(1,vel1)
            Set_Speed_rpm(2,vel2)
            Set_Speed_rpm(3,vel3)

            
        
except KeyboardInterrupt:
    Motion_x(0)
    pass    
Motion_x(0)    
plt.figure(1)
plt.plot(Pp1)
plt.hold
plt.plot(Pd1)
plt.figure(2)
plt.plot(Pp2)
plt.hold
plt.plot(Pd2)
plt.figure(3)
plt.plot(Pp3)
plt.hold
plt.plot(Pd3)

plt.figure(4)
plt.plot(VEL1)
plt.hold
plt.plot(VEL2)
plt.hold
plt.plot(VEL3)


#%%
position=0;
while True:
    Motion