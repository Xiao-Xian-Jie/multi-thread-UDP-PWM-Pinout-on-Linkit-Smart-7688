import mraa
import sys
import socket
import time
import math
from threading import Thread

UDP_IP = "192.168.100.1"
UDP_PORT = 12345

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

u=mraa.Uart(0)
u.setBaudRate(115200)
u.setMode(8, mraa.UART_PARITY_NONE, 1)
u.setFlowcontrol(False, False)

pwm0 = mraa.Pwm(18)
pwm0.period_us(700)
pwm0.enable(True)

out1 = mraa.Gpio(19)
out1.dir(mraa.DIR_OUT)

out2 = mraa.Gpio(14)
out2.dir(mraa.DIR_OUT)

global value
value = 0.3

# Define a function for the thread
def udpRX(delay):
 global value
 while True:
  my_bytes, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
  #u.writeStr("received message:")
  #u.writeStr(ord(my_bytes[1]))
  #u.writeStr("\n")
  #varA = ord(my_bytes[3])//ch3 0 - 100
  ch1 = ord(my_bytes[1])
  ch2 = ord(my_bytes[2])
  ch3 = ord(my_bytes[3])
  ch4 = ord(my_bytes[4])
  #add -var
  if ch1 > 110:
    ch1 = ch1 - 255
  if ch2 > 110:
    ch2 = ch2 - 255
  if ch4 > 110:
    ch4 = ch4 - 255
  #debug data
  u.writeStr("ch1:")
  u.writeStr(str(ch1))
  u.writeStr(" ch2:")
  u.writeStr(str(ch2))
  u.writeStr(" ch3:")
  u.writeStr(str(ch3))
  u.writeStr(" ch4:")
  u.writeStr(str(ch4))
  u.writeStr("\n")
  value = float(ch3)
  value = value/100
  pwm0.write(value)

def bilkTest(delay):
 while True:
  out1.write(1)
  time.sleep(1)
  out1.write(0)
  time.sleep(1)

def bilkTest2(delay):
 global value
 while True:
  out2.write(0)
  time.sleep(value)
  out2.write(1)
  time.sleep(value)

def i2cTest(delay):
 
  x = mraa.I2c(0)
  x.address(0x68)#MPU6050_ADDRESS         0x68
  # initialise device
  x.writeReg(0x6B, 0x00)#MPUREG_PWR_MGMT_1,BIT_H_RESET
  time.sleep(0.005)
  #set Sample rate = 1kHz
  x.writeReg(0x19, 0x00)#
  time.sleep(0.005)
  #set BITS_DLPF_CFG_98HZ
  x.writeReg(0x1A, 0x02)#
  time.sleep(0.005)
  # Full scale set to 2000 deg/sec
  x.writeReg(0x1B, 0x18)#
  time.sleep(0.005)
  #set MPUREG_ACCEL_CONFIG 2G
  x.writeReg(0x1C, 0x00)#
  time.sleep(0.005)
  #set MPUREG_INT_PIN_CFG 
  #x.writeReg(0x37, 0x02)#
  #time.sleep(0.005)
  #
  last_x_angle = 0.0
  last_y_angle = 0.0
  last_z_angle = 0.0
  last_gyro_x_angle = 0.0
  last_gyro_y_angle = 0.0
  last_gyro_z_angle = 0.0
  #
  while True:
    # read a 16bit reg, obviously it's uncalibrated so mostly a useless value :)
    #print(str(x.readWordReg(0xf6)))
    #x.readWordReg(0x3B)
    #read raw Acc
    ax = x.readReg(0x3B)<<8 | x.readReg(0x3C)# acc X
    if ax > 20000:
       ax = ax - 65535
    ay = x.readReg(0x3D)<<8 | x.readReg(0x3E)#acc Y
    if ay > 20000:
       ay = ay - 65535
    az = x.readReg(0x3F)<<8 | x.readReg(0x40)#acc Z
    #if az > 20000:
    #   az = az - 65535
    #read raw Gyro
    gx = x.readReg(0x43)<<8 | x.readReg(0x44)# gryo X
    if gx > 20000:
       gx = gx - 65535
    gy = x.readReg(0x45)<<8 | x.readReg(0x46)# gryo Y
    if gy > 20000:
       gy = gy - 65535
    gz = x.readReg(0x47)<<8 | x.readReg(0x48)# gryo Z
    if gz > 20000:
       gz = gz - 65535
    #u.writeStr("Start complementary")
    #u.writeStr("\n")
    #complementary 
    base_x_gyro = -35
    base_y_gyro = -1
    base_z_gyro = -29
    #ax = float(ax)
    #ay = float(ay)
    #az = float(az)
    #gx = float(ax)
    #gy = float(ay)
    #gz = float(az)
    gyro_x = (gx - base_x_gyro)/16.4
    gyro_y = (gy - base_y_gyro)/16.4
    gyro_z = (gz - base_z_gyro)/16.4
    accel_x = ax - 1100
    accel_y = ay - 300
    accel_z = az #- base_z_accel
    #
    accel_angle_y = math.atan(-1*accel_x/math.sqrt(math.pow(accel_y,2) + math.pow(accel_z,2)))*57.2958
    accel_angle_x = math.atan(accel_y/math.sqrt(math.pow(accel_x,2) + math.pow(accel_z,2)))*57.2958
    accel_angle_z = 0
    # Compute the (filtered) gyro angles
    dt = 0.010 #10ms
    gyro_angle_x = gyro_x*dt + last_x_angle
    gyro_angle_y = gyro_y*dt + last_y_angle
    gyro_angle_z = gyro_z*dt + last_z_angle
    #Compute the drifting gyro angles
    unfiltered_gyro_angle_x = gyro_x*dt + last_gyro_x_angle
    unfiltered_gyro_angle_y = gyro_y*dt + last_gyro_y_angle
    unfiltered_gyro_angle_z = gyro_z*dt + last_gyro_z_angle 
    #Apply the complementary filter to figure out the change in angle - choice of alpha is
    #estimated now.  Alpha depends on the sampling rate...
    alpha = 0.96
    angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x
    angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y
    angle_z = gyro_angle_z  #Accelerometer doesn't give z-angle 
    #Update the saved data with the latest values 
    last_x_angle = angle_x
    last_y_angle = angle_y
    last_z_angle = angle_z
    last_gyro_x_angle = unfiltered_gyro_angle_x
    last_gyro_y_angle = unfiltered_gyro_angle_y
    last_gyro_z_angle = unfiltered_gyro_angle_z
    # end complementary 
    #u.writeStr("angle_x :")
    #u.writeStr(str(angle_x))
    #u.writeStr("  angle_y :")
    #u.writeStr(str(angle_y))
    #u.writeStr("  angle_z :")
    #u.writeStr(str(angle_z))
    #u.writeStr("\n")
    time.sleep(0.010)#read very 10ms

t1 = Thread(target=udpRX, args=(1,))
t2 = Thread(target=bilkTest, args=(1,))
t3 = Thread(target=bilkTest2, args=(1,))
t4 = Thread(target=i2cTest, args=(1,))
t4.start()
t3.start()
t2.start()
t1.start()

  
   
