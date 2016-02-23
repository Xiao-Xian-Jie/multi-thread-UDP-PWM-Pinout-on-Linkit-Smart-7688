import mraa
import sys
import socket
import time
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
  data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
  u.writeStr("received message:")
  u.writeStr(data)
  u.writeStr("\n")
  value = float(data)
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

t1 = Thread(target=udpRX, args=(1,))
t2 = Thread(target=bilkTest, args=(1,))
t3 = Thread(target=bilkTest2, args=(1,))
t3.start()
t2.start()
t1.start()

  
   
