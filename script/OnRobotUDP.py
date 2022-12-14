import socket

import struct
import time
import numpy as np
from sympy import true

from AdmittanceController import AdmittanceController

PORT			= 49152	  # Port the Ethernet DAQ always uses 
SAMPLE_COUNT	= 10      # 10 incoming samples 
SPEED			= 10	  # 1000 / SPEED = Speed in Hz 
FILTER			= 4		  # 0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz 
BIASING_ON		= 0xFF    # Biasing on 
BIASING_OFF		= 0x00    # Biasing off 

COMMAND_START	= 0x0002  # Command for start streaming 
COMMAND_STOP	= 0x0000  # Command for stop streaming 
COMMAND_BIAS	= 0x0042  # Command for toggle biasing 
COMMAND_FILTER	= 0x0081  # Command for setting filter 
COMMAND_SPEED	= 0x0082  # Command for setting speed 
HEAD_FRAME  	= 0x1234  # Head Frame for sending command 

FORCE_DIV       = 10000.0 # Default divide value
TORQUE_DIV      = 100000.0# Default divide value


np.set_printoptions(precision=3, suppress=True)

def CreateSocket() -> socket.socket:
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def Addr(ipAddress, port):
    return (ipAddress, port)

def Connect(sck, ipAddress, port):
    addr = (ipAddress, port)
    sck.connect(addr)
    return True

def SendCommand(sck, command, data, addr):
    request = HEAD_FRAME.to_bytes(2,'big') \
               + command.to_bytes(2,'big') \
                  + data.to_bytes(4,'big')
    bytesToSend = bytearray(request)
    sck.sendto(bytesToSend, addr)

def Receive(sck) -> np.array:
    r = sck.recvfrom(36)
    # print(len(r[0]) ) # size must be 36

    data_bytes = r[0][12:36] 
    sensor_data = struct.unpack('>iiiiii', data_bytes) # big-endian, signed int(4 bytes)
    # TODO data checking 
    data = np.array(sensor_data, dtype=float)
    
    # Unit transfer (to N)
    data[:3] /= 10000. # default force divide value
    data[-3:] /= 100000. # default torque divide value
    return data

def Close(sck):
    sck.close()


if __name__ == '__main__':
    s = CreateSocket()
    addr = Addr("192.168.1.1", PORT)
    Connect(s, "192.168.1.1", PORT)

    # Set params
    SendCommand(s, COMMAND_SPEED, SPEED, addr)
    SendCommand(s, COMMAND_FILTER, FILTER, addr)
    SendCommand(s, COMMAND_BIAS, BIASING_ON, addr)

    # AdmittanceController
    Da = np.diag([20, 20, 20, 10, 10, 10])
    Ma = np.diag([15, 15, 15, 7.5, 7.5, 7.5])
    limit = np.array([0.5, 0.5, 0.5, 0.15, 0.15, 0.15]) # twist limit
    admtCtrl = AdmittanceController(Da, Ma, 100) 

    # Start streaming
    while True:
        SendCommand(s, COMMAND_START, SAMPLE_COUNT, addr)
        for i in range(SAMPLE_COUNT):
            data = Receive(s)
            print("\nidx ", i, "-> ", data)

            dt = admtCtrl.getDt()
            F_ext = data
            dxa = admtCtrl.computeAdmtOutput(F_ext)
            print("idx[{}], h: {}\ndxa: {}\n".format(i, admtCtrl.getAdmittanceRatio(), dxa) )

            time.sleep(dt)
