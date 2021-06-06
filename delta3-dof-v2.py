import timeit
import math
import time
import serial
import numpy as np
import struct
from numpy import linalg as LA
import matplotlib.pyplot as plt
import time
global offset
from simple_pid import PID

 
#%%
# Serial Defintion
ser4 = serial.Serial(port='COM10',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=.005, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None) 
ser2 = serial.Serial(port='COM11',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=.005, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None)
ser3 = serial.Serial(port='COM12',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=.005, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None) 
#ser1 = serial.Serial(port='COM6',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=.005, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None) 

e= (1/math.tan(np.deg2rad(30))) * 20
f= (1/math.tan(np.deg2rad(30))) * 26
re = 59.5
rf =  30.9
pi=3.1415
#ser4 = serial.Serial(port='COM6',baudrate=38400,parity='N',stopbits=1,bytesize=8,timeout=.005, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None)

# Check if Ports are opened
#print("port1 open =",ser4.isOpen(),"port2 open =",ser2.isOpen(),"port3 open =",ser3.isOpen(),"port4 open =",ser4.isOpen())

#%%
# --------------------------------Transport Protocol Functions---------------------------------------------
'''
Master Command:

'''


def CHKS_Kinco(value):
    # CHKS=-SUM(byte0:byte8)
    # CHKS is the lowesr byte of the calculation result:
    value = -1*sum(value)
    value = 2**32 + value  # ???
    C = list(struct.unpack('>4B', struct.pack('>L', value)))
    return C[3]


def HexCon(value):
    if value < 0:
        value = 2**32 + value

    H = list(struct.unpack('>4B', struct.pack('>L', value)))

    return H


def Write_reg(ID, value, CMD, index1, index2, Subindex):
    value_Hex = HexCon(value)
    s = [ID, CMD, index1, index2, Subindex, int(value_Hex[3]), int(
        value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1:
        ser4.write(s)
    elif ID == 2:
        ser2.write(s)
    elif ID == 3:
        ser3.write(s)

        
def Write_command(ID, value, CMD):
    value_Hex = HexCon(value)
    s = [ID, CMD, int(value_Hex[3]), int(value_Hex[2]),
         int(value_Hex[1]), int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1:
        ser4.write(s)
    elif ID == 2:
        ser2.write(s)
    elif ID == 3:
        ser3.write(s)


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

def Enable_all():
    Drive_Enable(1,1)
    Drive_Enable(2,1)
    Drive_Enable(3,1)

    Operation_Mode(1,-3)
    Operation_Mode(2,-3)
    Operation_Mode(3,-3)

    #Set speed rpms to 0 for safety issues
    Set_Speed_rpm(1,0)
    Set_Speed_rpm(2,0)
    Set_Speed_rpm(3,0)

def Disable_all():
    Drive_Enable(1,0)
    time.sleep(0.008)
    Drive_Enable(2,0)
    time.sleep(0.008)
    Drive_Enable(3,0)
    time.sleep(0.008)


# --------------------------------Operation_Mode-------------------------------
def Operation_Mode(ID, value):
    value_Hex = HexCon(value)
    s = [ID, 47, 96, 96, 0, int(value_Hex[3]), int(
        value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1:
        ser4.write(s)
    elif ID == 2:
        ser2.write(s)
    elif ID == 3:
        ser3.write(s)

# def write_command(ID, command):
#     cmd = command
#     cmd.append(CHKS_Kinco(command))
#     cmd = bytearray(cmd)
#     if ID == 1:
#         ser4.write(command)
#     elif ID == 2:
#         ser2.write(command)
#     elif ID == 3:
#         ser3.write(command)

# -----------------------------Enable Drivers------------------------------


def Drive_Enable(ID, value):

    if value == -1:      # Motor Power Off
        value = 0x06
    if value == 0:       # Motor Power Off
        value = 6
    elif value == 1:     # Enable Driver
        value = 0x2F
    elif value == 2:     # Start Absolute  position
        value = 0x103F
    elif value == 3:     # Start Relative position
        value = 0x105F
    elif value == 4:     # Start Torque and Speed mode
        value = 0x0F
    # elif value==5:       # Start Homing
    #     value=0x1F

# adj:

    value_Hex = HexCon(value)
    s = [ID, 43, 0x40, 96, 0, int(value_Hex[3]), int(
        value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1:
        ser4.write(s)
    elif ID == 2:
        ser2.write(s)
    elif ID == 3:
        ser3.write(s)

# ----------------------------------------Position----------------------------------------------------------

def Position_read():
    
    print("pp1=",Pos_Actual_r(1))
    time.sleep(.01)
    print("pp2=",Pos_Actual_r(2))
    time.sleep(.01)  
    print("pp3=",Pos_Actual_r(3))
    time.sleep(.008)  

def Position(ID, position, profile_speed):  # in degree
    # CONVERTING
    # position: deg ->
    # speed: RPM -> dec (probably)
    position = position*10000/360 * 50
    profile_speed = math.floor(profile_speed*2730.66 * 50)
    value_Hex = HexCon(int(position))
    s = [ID, 35, 0x7A, 96, 0, int(value_Hex[3]), int(value_Hex[2]), int(
        value_Hex[1]), int(value_Hex[0])]  # Position Mode
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    ser4.write(s)

    if profile_speed > 20:
        profile_speed = 20
    if profile_speed < -20:
        profile_speed = -20

    value_Hex = HexCon(profile_speed)
    s = [ID, 35, 0x81, 96, 0, int(value_Hex[3]), int(
        value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    ser4.write(s)

# -------------------------------Target_Toruqe---------------------------------


def Set_Torque_Nm(ID, Target_Torque_Nm):
    Target_Torque_Dec = math.floor(Target_Torque_Nm/2.668)*10
    value_Hex = HexCon(Target_Torque_Dec)
    s = [ID, 43, 113, 96, 0, int(value_Hex[3]), int(
        value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1:
        ser4.write(s)
    elif ID == 2:
        ser2.write(s)
    elif ID == 3:
        ser3.write(s)


# --------------------------------------speed_rpm---------------------------------------------


def Set_Speed_rpm(ID, Speed_rpm):
    if Speed_rpm > 10:
        Speed_rpm = 10
    if Speed_rpm < -10:
        Speed_rpm = -10
    # conversion:
    Speed_Dec = math.floor(Speed_rpm*2730.66 * 50)  # xxxxxxx
    value_Hex = HexCon(Speed_Dec)
    s = [ID, 35, 255, 96, 0, int(value_Hex[3]), int(
        value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1:
        ser4.write(s)
    elif ID == 2:
        ser2.write(s)
    elif ID == 3:
        ser3.write(s)



# -------------------------------Read Position, Velocity, Torque-----------------------------------


def Pos_Actual(ID):
    s = [ID, 64, 99, 96, 0, 0, 0, 0, 0]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    count=0
    if ID == 1:
        ser4.flushInput()
        ser4.write(s)
        pos = list(ser4.read(10))
        while len(pos)!=10:
            pos = list(ser4.read(10))
            count=count+1
            print("{}times tried to read pos but failed from ID={}".format(count,ID))
            if count==10:
                print("Unable to read data from ID={}".format(ID))
                break
    elif ID == 2:
        ser2.flushInput()
        ser2.write(s)
        pos = list(ser2.read(10))
        while len(pos)!=10:
            pos = list(ser2.read(10))
            count=count+1
            print("{}times tried to read pos but failed from ID={}".format(count,ID))
            if count==10:
                print("Unable to read data from ID={}".format(ID))
                break
    elif ID == 3:
        ser3.flushInput()
        ser3.write(s)
        pos = list(ser3.read(10))
        while len(pos)!=10:
            pos = list(ser3.read(10))
            count=count+1
            print("{}times tried to read pos but failed from ID={}".format(count,ID))
            if count==10:
                print("Unable to read data from ID={}".format(ID))
                break
 
    pos = value_read(pos)
    return pos/50


def Offset():
    global offset
    p1 = Pos_Actual_r(1)
    p2 = Pos_Actual_r(2)
    p3 = Pos_Actual_r(3)
    offset=[p1, p2, p3]
    
def check_valid_data(data):
    if len(data) != 10:
        return 0
    
def Stop():
    Set_Speed_rpm(1,0)
     
    Set_Speed_rpm(2,0)
       
    Set_Speed_rpm(3,0)
    


def Velocity_Actual(ID):

    s = [ID, 64, 108, 96, 0, 0, 0, 0, 0]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1:
        ser4.flushInput()
        ser4.write(s)
        vel = list(ser4.read(10))
    elif ID == 2:
        ser2.flushInput()
        ser2.write(s)
        vel = list(ser2.read(10))
    elif ID == 3:
        ser3.flushInput()
        ser3.write(s)
        vel = list(ser3.read(10))


    vel = value_read_vel(vel)
    return vel/50


def Torque_Actual(ID):
    s = [ID, 64, 119, 96, 0, 0, 0, 0, 0]
    s.append(CHKS_Kinco(s))
    s = bytearray(s)
    if ID == 1:
        ser4.flushInput()
        ser4.write(s)
        torque = list(ser4.read(10))
    elif ID == 2:
        ser2.flushInput()
        ser2.write(s)
        torque = list(ser2.read(10))
#        print("torque2 old =", torque2)
#        torque2 = list(ser4.read(10))
#        print("torque2 =", torque2)
#        return value_read_torque(torque2)

    elif ID == 3:
        ser3.flushInput()
        ser3.write(s)
        torque = list(ser3.read(10))

    return value_read_torque(torque)


def Pos_Actual_r(ID):
    global offset
    if 'offset' not in globals():
        offset = [0, 0, 0]
#    start=timeit.default_timer()
    pp = Pos_Actual(ID)
#    print("__",timeit.default_timer()-start)
    i = 0
#    start=timeit.default_timer()
    while pp == -333333:
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
        if i > 20:
            Stop()
            Disable_all()
            break
#    print("__**__",timeit.default_timer()-start)
    return pp - offset[ID-1]


def Velocity_Actual_r(ID):
    # start=timeit.default_timer()
    vel = Velocity_Actual(ID)
    # print("__",timeit.default_timer()-start)
    i = 0
    while vel == -666666:
        vel = Velocity_Actual(ID)
        i += 1
#        print(i,ID)
        if i > 20:
            Stop()
            Disable_all()
            break
#    print(i)
    return vel

def angle_yz(x0, y0, z0, theta=None): #### ????
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
        status = angle_yz((x0)*np.cos(np.pi) + (y0)*np.sin(np.pi),(y0)*np.cos(np.pi)-(x0)*np.sin(np.pi),(z0), theta2)
    if status[0] == 0:
        theta2 = status[1]
        status = angle_yz((x0)*np.cos(np.pi) - (y0)*np.sin(np.pi),(y0)*np.cos(np.pi) + (x0)*np.sin(np.pi),(z0),theta3)
    theta3 = status[1]

    return status[0],theta1,theta2,theta3





def Motion_z(speed):
    Set_Speed_rpm(1,speed)
    Set_Speed_rpm(2,speed)
    Set_Speed_rpm(3,speed)
    
#    #duplicate code to make sure command has been obeyed
    Set_Speed_rpm(1,speed)
    Set_Speed_rpm(2,speed)
    Set_Speed_rpm(3,speed)
    
    
def Motion_y(speed):
    Set_Speed_rpm(1,-1*speed)
    Set_Speed_rpm(2,speed)
    Set_Speed_rpm(3,speed)

def Motion_x(speed):
    Set_Speed_rpm(1,speed)
    Set_Speed_rpm(2,speed)
    Set_Speed_rpm(3,-1*speed)
#%%--------------------new pid controller-----------------
#def p_Controller(set_point_angle,ID):
#    Kp=0.05
#    present_position=Pos_Actual_r(ID)
#    error=set_point_angle-present_position
#    system_input=Kp*(error)
#    return system_input
#PID LIBRARY
import datetime

class PID:
    def __init__(self,Kp,Ki,Kd,setPoint=0,SampleTime=100): #sample time is in millisconds
        if Kp<0 or Kd<0 or Ki<0:
            print("invalid pid constants")
            return
        self.SampleTime=SampleTime
        sampleTimeInSec=SampleTime/1000 
        self.kp=Kp
        self.ki=Ki*sampleTimeInSec
        self.kd=Kd/sampleTimeInSec
        self.lastError=0
        self.integralTerm=0 #used for I term
        self.lastTime=datetime.datetime.now()
#        startTime=startTime.seconds()*1000
        self.minOutput=0
        self.maxOutput=0
        self.Error=0


    def Compute(self,feedback):
        presentTime=datetime.datetime.now()
        timeChange=(presentTime-self.lastTime).total_seconds()*1000

        if timeChange>self.SampleTime: #if a time interval equal to sample time has passed
            #Compute Working Error Variables
            self.Error=self.setPoint-feedback
            dError=self.Error-self.lastError #error- last error
            self.integralTerm=self.integralTerm+self.ki*self.Error
            derivativeTerm=self.kd*dError
            proportionalTerm=self.kp*self.Error
            PIDOutput=self.integralTerm+derivativeTerm+proportionalTerm

            if self.maxOutput!=0 and PIDOutput>self.maxOutput:
                PIDOutput=self.maxOutput
            elif self.minOutput!=0 and PIDOutput<self.minOutput:
                PIDOutput=self.minOutput
            return PIDOutput
        self.lastTime=presentTime
            

    def SetSampleTime(self,newSampleTime):
        ratio=newSampleTime/self.SampleTime
        self.ki=self.ki*ratio
        self.kd=self.kd/ratio
        self.SampleTime=newSampleTime

    def SetOutputLimits(self,minOut,maxOut):
        self.minOutput=minOut
        self.maxOutput=maxOut
        
    def DefineSetpoint(self,coord):
        self.setPoint=coord
        
        
    def set_PID_constants(self,Kp,Ki,Kd):
        self.kp=Kp
        self.ki=Ki
        self.kd=Kd
        

pids=[]
kp = 0.1
ki = 0.01
kd = 0.01
#3 pids for 3 angles

positionReadFailed=0

pid1=PID(kp,ki,kd)
pid2=PID(kp,ki,kd)
pid3=PID(kp,ki,kd)

def implement_PID(set_point_coord,feedback):
    controllerOutput=[]
    #converting xyz coord to angle by inverse kinematics
    E,theta1,theta2,theta3=Inverse(set_point_coord[0],set_point_coord[1],set_point_coord[2])
    #system input is the return value of controller
    print("thetaas : 1: {}, 2: {}, 3: {} \n".format(theta1,theta2,theta3))
    pid1.DefineSetpoint(theta1)
    pid2.DefineSetpoint(theta2)
    pid3.DefineSetpoint(theta3)
    
#    in1=Pos_Actual_r(1)
#    in2=Pos_Actual_r(2)
#    in3=Pos_Actual_r(3)
#    print("ins: {}, {} , {} ".format(in1,in2,in3))
#    if not (in1>=0) or not (in2>=0) or not (in3>=0):
#        return positionReadFailed

    controllerOutput.append(pid1.Compute(feedback[0]))
    controllerOutput.append(pid2.Compute(feedback[1]))
    controllerOutput.append(pid3.Compute(feedback[2]))
    
    print("controlleroutput : 1: {}, 2: {}, 3: {} \n".format(controllerOutput[0],controllerOutput[1],controllerOutput[2]))

    return controllerOutput
#%%-
set_point_coo=[10, -15, -70] #cm

#end_time=time.time()+0.15*60
# time.time()<end_time

def go_to(set_point_coord):
    ID1_arrived=0
    ID2_arrived=0
    ID3_arrived=0
    all_arrived=0  
    while not all_arrived:

        in1=Pos_Actual_r(1)
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        in3=Pos_Actual_r(3)
        feedback=[in1,in2,in3]
        system_input=implement_PID(set_point_coord,feedback) #system input=controller output
        
        if not system_input[0] or not system_input[1] or not system_input[2]:
            print("FAILed")
            break
        a=system_input[0]
        b=system_input[1]
        c=system_input[2]
        
        print("SystemInputs: 1: {}, 2: {}, 3: {} \n".format(a,b,c))
        print("Error:{} \n ".format(pid1.Error))
        
        Set_Speed_rpm(1,system_input[0])
        Set_Speed_rpm(2,system_input[1])
        Set_Speed_rpm(3,system_input[2])
        
    
        if np.abs(pid1.Error)<1 :
            Set_Speed_rpm(1,0)
            Set_Speed_rpm(1,0)
            ID1_arrived=1
            print("ID 1 ARRIVED!")
        if np.abs(pid2.Error)<1 :
            Set_Speed_rpm(2,0)
            Set_Speed_rpm(2,0)
            ID2_arrived=1
            print("ID 2 ARRIVED!")
        if np.abs(pid1.Error)<1 :
            Set_Speed_rpm(3,0)
            Set_Speed_rpm(3,0)
            ID3_arrived=1
            print("ID 3 ARRIVED!")
        if ID1_arrived and ID2_arrived and ID3_arrived:
            all_arrived=1
            print("ALL ARRIVED!")
            break
    #    Set_Speed_rpm(1,a)
    
    Motion_z(0)
    Motion_z(0)



#    Set_Speed_rpm(2,b)
#    Set_Speed_rpm(3,c)
#    if time.time()==end_time:
#        Motion_z(0)
#        break
#position=0
#while position!=20:
#    Motion_z(0.1)
#    position=Pos_Actual_r(1)
#    if position==20:
#        break
#Motion_z(0)
#%%
position=0;
for i in range(10):
    go_to([10,10,-65])
    go_to([-10,-10,-70])
    
#%% circling
def circle_motion(center,radi,height,n,period):  #position and lengths in cm -- time in seconds
    first_time=datetime.datetime.now()
    dtime=0
    while dtime<n*period:
        last_time=datetime.datetime.now()
        dtime=(last_time-first_time).total_seconds()
        px = radi * np.cos(np.pi*2/period*dtime)+center[0] 
        py = radi * np.sin(np.pi*2/period*dtime)+center[1]
        pz=height
#        go_to([px,py,pz])
####
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        feedback=[in1,in2,in3]
        system_input=implement_PID([px,py,pz],feedback) 
        Set_Speed_rpm(1,system_input[0])
        Set_Speed_rpm(2,system_input[1])
        Set_Speed_rpm(3,system_input[2])

####

        if dtime==n*period:
            print("encircled {} times!".format(n))
            Motion_z(0)
            Motion_z(0)
            break
    Motion_z(0)
    Motion_z(0)

# spiral
def spiral_motion(center,radi,starting_height,ending_height,n,period):
    pitch=(ending_height-starting_height)/n
    e,x,y,z=Forward(Pos_Actual_r(1),Pos_Actual_r(2),Pos_Actual_r(3))
    first_time=datetime.datetime.now()
    z_velocity=pitch/period
    dtime=0
    while not z==ending_height and dtime<n*period :
        current_time=datetime.datetime.now()
        dtime=(current_time-first_time).total_seconds()
        
        px=radi*np.cos(np.pi*2/period*dtime)+center[0] 
        py=radi*np.sin(np.pi*2/period*dtime)+center[1]
        pz=starting_height+dtime*z_velocity
####
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        in1=Pos_Actual_r(1) #Just in case that didn't work!
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)

        feedback=[in1,in2,in3]
        system_input=implement_PID([px,py,pz],feedback) 
        
        Set_Speed_rpm(1,system_input[0])
        Set_Speed_rpm(2,system_input[1])
        Set_Speed_rpm(3,system_input[2])

####
        e,x,y,z=Forward(in1,in2,in3)
        if z==ending_height or dtime==n*period:
            print("End Of Spiral!")
            Motion_z(0)
            Motion_z(0)
            break
    Motion_z(0)
    Motion_z(0)        
    
# ellipse
MINOR_ON_X=0
def elliptic_motion(center,minor_axis,major_axis,height,n,period,orientation):
    if orientation==MINOR_ON_X:
        x_radii=minor_axis
        y_radii=major_axis
        z_radii=0
    else:
        y_radii=minor_axis
        x_radii=major_axis
        z_radii=0
    first_time=datetime.datetime.now()
    dtime=0
    while dtime<n*period:
        last_time=datetime.datetime.now()
        dtime=(last_time-first_time).total_seconds()
        px = x_radii * np.cos(np.pi*2/period*dtime)+center[0] 
        py = y_radii * np.sin(np.pi*2/period*dtime)+center[1]
        pz=height
#        go_to([px,py,pz])
####
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        feedback=[in1,in2,in3]
        system_input=implement_PID([px,py,pz],feedback) 
        Set_Speed_rpm(1,system_input[0])
        Set_Speed_rpm(2,system_input[1])
        Set_Speed_rpm(3,system_input[2])

####

        if dtime==n*period:
            print("encircled {} times!".format(n))
            Motion_z(0)
            Motion_z(0)
            break
    Motion_z(0)
    Motion_z(0)

#%% circle (ellipse-based)
def circle_motion_mod(center,radi,height,n,period):
    elliptic_motion(center,radi,radi,height,n,period,1)
    Motion_z(0)
#%%
circle_motion([0,0],10,-75,5,8) #arguments: center,minor_axis,major_axis,height,n,period
Motion_z(0)
#%%
spiral_motion([0,0],8,-75,-50,7,5) # arguments: center,radi,starting_height,ending_height,n,period
Motion_z(0)
Motion_z(0)
#%%
elliptic_motion([0,0],5,15,-55,5,2.5,MINOR_ON_X) # arguments: center,minor_axis,major_axis,height,n,period,orientation
Motion_z(0)
#%%
circle_motion_mod([0,0],10,-75,5,2.5)
#%%
 go_to_planned([0,0,-85],2.5)

#%%
import matplotlib.pyplot as plt
def PPO(starting_coord,ending_coord,rise_time):
    start_time=datetime.datetime.now()
#    go_to(starting_coord)
    dz=ending_coord[2]-starting_coord[2]
    dtime=0
    curve_time=1
    z_poses=[]
    y_poses=[]
    time=[]
    y_pos=0
    vertical_motion_time=rise_time-curve_time
    while dtime<rise_time:
        last_time=datetime.datetime.now()
        dtime=(last_time-start_time).total_seconds()
        if dtime>=rise_time:
#            Motion_z(0)
#            Motion_z(0)
            break

        tau=dtime/rise_time
        ytau=(dtime-vertical_motion_time)/curve_time
        
#        z_pos=-20*(tau**7)+70*(tau**6)-84*(tau**5)+35*(tau**4) #using 4-5-6-7 trajectory
        z_pos=trajectory_4567(rise_time,0,start_time)
        z_pos=z_pos*dz+starting_coord[2]
        
        
        if dtime>=vertical_motion_time:
#            y_pos=-20*(ytau**7)+70*(ytau**6)-84*(ytau**5)+35*(ytau**4)
            y_pos=trajectory_4567(curve_time,vertical_motion_time,start_time)
            y_pos=y_pos*10+starting_coord[1]
        y_poses.append(y_pos)
        
####    
        px=0
        py=y_pos
        pz=z_pos
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        feedback=[in1,in2,in3]
        system_input=implement_PID([px,py,pz],feedback) 
        Set_Speed_rpm(1,system_input[0])
        Set_Speed_rpm(2,system_input[1])
        Set_Speed_rpm(3,system_input[2])

####
        z_poses.append(z_pos)
        time.append(dtime)
        
        curve_time=1
        
        
#    plt.plot(time,y_poses)
    plt.plot(y_poses,z_poses)
    
    Motion_z(0)
#%%
def go_to_planned(ending_coord,duration): #moving between two points using 4567 interpolation 
    start_time=datetime.datetime.now()

    current_position=[0,0,0]
    last_position=[0,0,0]
    distance=[0,0,0]

    for i in range(3):
        current_position[i]=Pos_Actual_r(i+1)
    
    e,current_position[0],current_position[1],current_position[2]=Forward(Pos_Actual_r(1),Pos_Actual_r(2),Pos_Actual_r(3))
    
    for i in range(3):
        distance[i]=ending_coord[i]-current_position[i]
    
    
    dtime=0
    time=[]
    x_poses=[]
    y_poses=[]
    z_poses=[]
    while dtime<duration:
        last_time=datetime.datetime.now()
        dtime=(last_time-start_time).total_seconds()
        if dtime>=duration:
            Motion_z(0)
            Motion_z(0)
            break

        tau=dtime/duration
        
        pos_change=trajectory_4567(duration,0,start_time)
        
        for i in range(3):
            last_position[i]=pos_change*distance[i]+current_position[i]
     
        x_poses.append(last_position[0])
        y_poses.append(last_position[1])
        z_poses.append(last_position[2])
        time.append(dtime)

####
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        feedback=[in1,in2,in3]
        system_input=implement_PID(last_position,feedback) 
        Set_Speed_rpm(1,system_input[0])
        Set_Speed_rpm(2,system_input[1])
        Set_Speed_rpm(3,system_input[2])

####
#        z_poses.append(z_pos)
                
        
#    plt.plot(time,y_poses)
    plt.plot(time,x_poses)
    #%%
#for i in range(10):
#    go_to_planned([0,5,-45],0.5)
go_to_planned([0,0,-85],3)
#Motion_z(0)
#%%
def trajectory_4567(time_map,time_shift,start_time): #return value within 0 , 1
    last_time=datetime.datetime.now()
    dtime=(last_time-start_time).total_seconds()
    tau=(dtime-time_shift)/time_map
    s=-20*(tau**7)+70*(tau**6)-84*(tau**5)+35*(tau**4)
    return s
def trajectory_via_points(start_time,velocity,):
    last_time=datetime.datetime.now()
    t=(last_time-start_time).total_seconds()
    
#%% i
        go_to_planned([0,0,-85],2.5)
        
        PPO([-5,0,-85],[0,0,-55],2.5)
#
#%%
def go_through(points,velocity):
    x_points=[]
    y_points=[]
    z_points=[]
    for i in range(len(points)):
        x_points.append(points[i][0])
        y_points.append(points[i][1])
        z_points.append(points[i][2])
        
    start_time=datetime.datetime.now()
    A_x=produce_BC(start_time,velocity,x_points)
    A_y=produce_BC(start_time,velocity,y_points)
    A_z=produce_BC(start_time,velocity,z_points)

    vel_accel_ans=[0,0,0,0]
    B_x=x_points.append(vel_accel_ans)
    B_y=y_points.append(vel_accel_ans)
    B_z=z_points.append(vel_accel_ans)

    x_coeffs=np.linalg.inv(A_x).dot(B_x) #A*X=B -> X=A^-1 * B
    y_coeffs=np.linalg.inv(A_y).dot(B_y)
    z_coeffs=np.linalg.inv(A_z).dot(B_z)
    
    cartesian_position=[0,0,0]
    while not(position[0]==x_points[0] and position[1]==y_points[0] and position[2]==z_points[0]):
        last_time=datetime.datetime.now()
        dtime=(last_time-start_time).total_seconds()
        if dtime>=duration:
            Motion_z(0)
            Motion_z(0)
            break        

        x=produce_polynomial(x_coeffs,dtime)
        y=produce_polynomial(y_coeffs,dtime)
        z=produce_polynomial(z_coeffs,dtime)
             
        x_poses.append(x)
        y_poses.append(y)
        z_poses.append(z)
        time.append(dtime)
        
        last_position=[x,y,z]

####
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        in1=Pos_Actual_r(1)
        in2=Pos_Actual_r(2)
        in3=Pos_Actual_r(3)
        feedback=[in1,in2,in3]
        system_input=implement_PID(last_position,feedback) 
#        Set_Speed_rpm(1,system_input[0])
#        Set_Speed_rpm(2,system_input[1])
#        Set_Speed_rpm(3,system_input[2])
        

    plot(time,x_poses)

#%% Use this section to calculate params required for function go_through
def produce_polynomial(coeffs_matrix,t):
    n=len(coeffs_marix)-1
    poly=0
    for i in range(len(coeffs_matrix)):
        poly=poly+coeffs[i]*t**(n-i)
    
    return poly
def produce_BC(start_time,velocity,points): #produces array including the polynomials each BC creates
    last_time=datetime.datetime.now()
    t=(last_time-start_time).total_seconds()
    timing=time_schedule(velocity,points)
    BC_2D_array=[];
    n=len(points)+4

    for i in range(len(points)):
        variable_powers=[]
        for j in range(n):
            variable_powers.append(timing[i]**(n-i))
        BC_2D_array.append(variable_powers)
    #velocity conditions
    variable_powers=[]
    for j in range(n-1):
        variable_powers.append((n-i)*0**(n-1-i))
    variable_powers.append(0)
    BC_2D_array.append(variable_powers)
    variable_powers=[]
    for j in range(n-1):
        variable_powers.append((n-i)*timing[-1]**(n-1-i))
    variable_powers.append(0)
    BC_2D_array.append(variable_powers)
   #acceleration conditions     
    variable_powers=[]
    for j in range(n-2):
        variable_powers.append((n-i)*(n-i-1)*0**(n-2-i))
    variable_powers.append(0)
    variable_powers.append(0)
    BC_2D_array.append(variable_powers)
    variable_powers=[]
    for j in range(n-1):
        variable_powers.append((n-i)*(n-i-1)*timing[-1]**(n-2-i))
    variable_powers.append(0)
    variable_powers.append(0)
    BC_2D_array.append(variable_powers)
    
    return variable_powers

def calc_distance(f_point,s_point):

    return math.sqrt((f_point[0]-s_point[0])**2+(f_point1[1]-s_point[1])**2+(f_point[2]-s_point[2])**2)

def time_schedule(velocity,points):
    schedule=[]
    start_point=points[0]
    for i in range(len(points)):
        print("length point {}\n".format(len(points)))
        current_point=points[i]
#        print(len(current_point))
        dt=calc_distance(current_point,start_point)/velocity #dt=dx/v or...
        schedule.append(dt)
    return schedule
  #%%

go_through([[0,0,0],[1,2,4]],2) # arguments : points,velocity  
#%%
        
Motion_z(0)
#%% 
def return_current_xyz(): # return current position in cartesian coord
    thetas=[]
    for i in range(1,3):
        thetas.append(Pos_Actual_r(i))
    e,x,y,z=Forward(thetas[0],thetas[1],thetas[2])
    return [x,y,z]
def distance(first_p,second_p): # return distance between two points in space
    squares_sum=0
    for i in range(3):
       squares_sum=squares_sum+(second_p[i]-first_p[i])**2
    return (squares_sum)**0.5
      
#%% motion planner
import datetime
from scipy import linalg
class Motion_Planner:
    def __init__(self,vel_conds,accel_conds,points,velocity):
        self.points=points
        self.times=self.schedule_planner(velocity)
        self.group_points()
        
        #keeping the history of each set of points
        self.last_x_accel=0
        self.last_x_vel=0
        self.last_x_pos=points[0][0]
        
        self.last_y_accel=0
        self.last_y_vel=0
        self.last_y_pos=points[0][1]
        
        self.last_z_accel=0
        self.last_z_vel=0
        self.last_z_pos=points[0][2]
        
#        return_current_xyz()
        
    def group_points(self,k=4):
        self.point_sets=[]
        new_point_set=[]
        self.time_sets=[]
        new_time_set=[]
        index=[0,1,2,3]
        
        self.x_conds=[]
        self.y_conds=[]
        self.z_conds=[]
        x_new=[]
        y_new=[]
        z_new=[]
    
        while not (index[-1]>(len(self.points))):
            for i in range(k):
                new_point_set.append(self.points[index[i]])
                new_time_set.append(self.times[index[i]])
                
                x_new.append(self.points[index[i]][0])
                y_new.append(self.points[index[i]][1])
                z_new.append(self.points[index[i]][2])


            self.point_sets.append(new_point_set)
            self.time_sets.append(new_time_set)
            
            self.x_conds.append(x_new)
            self.y_conds.append(y_new)
            self.z_conds.append(z_new)
            #clear temps
            new_time_set=[]
            new_point_set=[]
            x_new=[]
            y_new=[]
            z_new=[]
            
            for i in range(k):
                index[i]=index[i]+k-1
                
    def schedule_planner(self,velocity): #time distribution (to be improved)
        times=[0]
        for i in range(len(self.points)-1):
            times.append((i+1)*(distance(self.points[i],self.points[i+1])/velocity)) #dt=dx/velocity where dx is the distance within 2 points
        return times   
            
    def generate_poly_terms(self,first_time,k,set_number):
        times=self.time_sets[set_number]
        last_time=datetime.datetime.now()
        t=(last_time-first_time).total_seconds()
        poly_terms=[]
        for i in range(0,k):
            new_line=[]
            for j in range(k+2):
                if j==0:
                    new_line.append(0)
                else:
                    new_line.append((times[i]-times[0])**j)
            poly_terms.append(new_line)
            new_line=[]
        return poly_terms

    def generate_derivative_poly_terms(self,first_time,k,set_number):
        times=self.time_sets[set_number]
        last_time=datetime.datetime.now()
        t=(last_time-first_time).total_seconds()
        poly_terms=[]
        for i in range(0,k):
            new_line=[]
            for j in range(k+2):
                if j==0:
                    new_line.append(0)
                else:
                    new_line.append((j)*(times[i]-times[0])**(j-1))
            poly_terms.append(new_line)

        return poly_terms
    
    def generate_2nderivative_poly_terms(self,first_time,k,set_number):
        times=self.time_sets[set_number]
        last_time=datetime.datetime.now()
#        t=(last_time-first_time).total_seconds()
        poly_terms=[]
        for i in range(0,k):
            new_line=[]
            for j in range(k+2):
                if j==0 or j==1:
                    new_line.append(0)
                else:
                    new_line.append((j)*(j-1)*(times[i]-times[0])**(j-2))
            poly_terms.append(new_line)
        return poly_terms
    
    def produce_coefficients(self,k,set_number): #A*X=B 
        points=self.point_sets[set_number]
        first_time=datetime.datetime.now()
        poly_terms=self.generate_poly_terms(first_time,k,set_number) #A
        deriv_poly_terms=self.generate_derivative_poly_terms(first_time,k,set_number)
        seconderiv_poly_terms=self.generate_2nderivative_poly_terms(first_time,k,set_number)
        
        #coefficients for equations system
        A=[]
        A.append(deriv_poly_terms[0])
        A.append(seconderiv_poly_terms[0])
        for i in range(len(poly_terms)):
            A.append(poly_terms[i])

        # BC 
        # 3 systems of linear equations are to be solved for each of the 3 dimensions which only vary in boundary\initial conds
        x_B_conds=[self.last_x_vel,self.last_x_accel]
        y_B_conds=[self.last_y_vel,self.last_y_accel]
        z_B_conds=[self.last_z_vel,self.last_z_accel]
        for i in range(k):
            x_B_conds.append(self.x_conds[set_number][i])

        for j in range(k):
            y_B_conds.append(self.y_conds[set_number][j])
    
        for m in range(k):
            z_B_conds.append(self.z_conds[set_number][m])
            
        A_array=np.array(A)
        x_B_array=np.array(x_B_conds)
        y_B_array=np.array(y_B_conds)
        z_B_array=np.array(z_B_conds)
        
        x_solved_poly_coeffs=self.solve_linear_system(A_array,x_B_conds)
        y_solved_poly_coeffs=self.solve_linear_system(A_array,y_B_conds)
        z_solved_poly_coeffs=self.solve_linear_system(A_array,z_B_conds)
        return x_solved_poly_coeffs,y_solved_poly_coeffs,z_solved_poly_coeffs
#        return X
        
    def solve_linear_system(self,coeff_array,conds):
        A_inv=linalg.pinv2(np.asmatrix(coeff_array))
        B=np.transpose(np.asmatrix(conds))
        X=A_inv*B #coefficients matrix
        return X
    
    def calc_polynomial(self,coeffs_matrix,t,set_number):
        t_init=self.time_sets[set_number][0]
        n=len(coeffs_matrix)
        poly=0
        for i in range(len(coeffs_matrix)):
            poly=poly+coeffs_matrix[i]*(t-t_init)**i
        return poly
    
    def calc_deriv_polynomial(self,coeffs_matrix,t,set_number):
        t_init=self.time_sets[set_number][0]
        n=len(coeffs_matrix)
        poly=0
        for i in range(len(coeffs_matrix)):
            if i==0:
                poly=poly+0
            else:
                poly=poly+coeffs_matrix[i]*i*(t-t_init)**(i-1)
        return poly

    def calc_2nderiv_polynomial(self,coeffs_matrix,t,set_number):
        t_init=self.time_sets[set_number][0]
        n=len(coeffs_matrix)
        poly=0
        for i in range(len(coeffs_matrix)):
            if i==0 or i==1:
                poly=poly+0
            else:
                poly=poly+coeffs_matrix[i]*i*(i-1)*(t-t_init)**(i-2)
        return poly
    
    def solve(self,set_number,t,k=4):
        x_coeffs,y_coeffs,z_coeffs=self.produce_coefficients(k,set_number)
        self.x_to_go=self.calc_polynomial(x_coeffs,t,set_number)
        self.y_to_go=self.calc_polynomial(y_coeffs,t,set_number)
        self.z_to_go=self.calc_polynomial(z_coeffs,t,set_number)
        
        self.update_planner(x_coeffs,y_coeffs,z_coeffs,set_number)
#        X=[float(self.x_to_go),float(self.y_to_go),float(self.z_to_go)]
#        return X
    
    def update_planner(self,x_coeffs,y_coeffs,z_coeffs,set_number):
        last_time=self.time_sets[set_number][-1]        
        self.last_x_vel=self.calc_deriv_polynomial(x_coeffs,last_time,set_number)
        self.last_x_accel=self.calc_2nderiv_polynomial(x_coeffs,last_time,set_number)
        self.last_x_pos=self.x_conds[set_number][-1]
        
        self.last_y_vel=self.calc_deriv_polynomial(y_coeffs,last_time,set_number)
        self.last_y_accel=self.calc_2nderiv_polynomial(y_coeffs,last_time,set_number)
        self.last_y_pos=self.y_conds[set_number][-1]

        self.last_z_vel=self.calc_deriv_polynomial(z_coeffs,last_time,set_number)
        self.last_z_accel=self.calc_2nderiv_polynomial(z_coeffs,last_time,set_number)
        self.last_z_pos=self.z_conds[set_number][-1]        
        
    

plan1=Motion_Planner([],[],[[1,0,0],[2,0,0],[3,0,0],[4,0,0],[5,0,0],[6,0,0],[7,0,0],[8,0,0],[9,0,0],[10,0,0]],5)
#ps,ts=plan1.group_points()
t=0.45

ts=[]
xs=[]

#print(ts)
#plt.plot(ts,xs)
first_time=datetime.datetime.now()

ts=[]
xs=[]
#while t<5:
#    last_time=datetime.datetime.now()
#    t=(last_time-first_time).total_seconds()
#    ts.append(t)
#    x=plan1.solve(0,t)
#    xs.append(x[0])
#plt.plot(ts,xs)
#    
plan1.solve(1,t)

#%% grouped points trajectoryplanning test No.1
points=[[1,0,0],[2,0,0],[3,0,0],[4,0,0],[5,0,0],[6,0,0],[7,0,0],[8,0,0],[9,0,0],[10,0,0]]
def planned_motion_grouped(points):
    plan=Motion_Planner([],[],points,5)
    
    start_time=datetime.datetime.now()
    
    current_position=[1,0,0]
    last_position=[0,0,0]
#    distance=[0,0,0]

#    for i in range(3):
#        current_position[i]=Pos_Actual_r(i+1)
    
#    e,current_position[0],current_position[1],current_position[2]=Forward(Pos_Actual_r(1),Pos_Actual_r(2),Pos_Actual_r(3))
    
#    for i in range(3):
#        distance[i]=ending_coord[i]-current_position[i]

    
    dtime=0
    time=[]
    x_poses=[]
    y_poses=[]
    z_poses=[]
    for set_number in range(3):
        while not last_position[0]==plan.point_sets[set_number][-1][0]: #comparing the last pos with the current pos in each set of points
            last_time=datetime.datetime.now()
            t=(last_time-start_time).total_seconds()
            if last_position[0]==plan.point_sets[set_number][-1][0]:
                Motion_z(0)
                Motion_z(0)
                break
            plan.solve(set_number,t)
            last_position=[plan.x_to_go,plan.y_to_go,plan.z_to_go]
            
            x_poses.append(last_position[0])
            y_poses.append(last_position[1])
            z_poses.append(last_position[2])
            time.append(t)
    
####
#            in1=Pos_Actual_r(1)
#            in2=Pos_Actual_r(2)
#            in3=Pos_Actual_r(3)
#            in1=Pos_Actual_r(1)
#            in2=Pos_Actual_r(2)
#            in3=Pos_Actual_r(3)
#            feedback=[in1,in2,in3]
#            system_input=implement_PID(last_position,feedback) 
#            Set_Speed_rpm(1,system_input[0])
#            Set_Speed_rpm(2,system_input[1])
#            Set_Speed_rpm(3,system_input[2])

####
#        z_poses.append(z_pos)
                
        
#    plt.plot(time,y_poses)
    plt.plot(time,x_poses)
planned_motion_grouped(points)