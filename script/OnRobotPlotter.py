from QtPlotter import QtPlotter
from OnRobotUDP import *

#filter
import scipy.signal

class OnRobotPlotter(QtPlotter):
    def __init__(self, sampleinterval=0.1, timewindow=10., size=(800,600), dict_data=[]):
        super().__init__(sampleinterval, timewindow, size, dict_data)
        self.initSensor()

    def initSensor(self):
        print('---OnRobotUDP Init---')
        self.s_ = CreateSocket()  
        self.addr_ = Addr("192.168.1.1", PORT)
        Connect(self.s_, "192.168.1.1", PORT)
        # Set params
        SendCommand(self.s_, COMMAND_SPEED, SPEED, self.addr_)
        SendCommand(self.s_, COMMAND_FILTER, FILTER, self.addr_)
        SendCommand(self.s_, COMMAND_BIAS, BIASING_ON, self.addr_)

    
    def updateplot(self):
        SendCommand(self.s_, COMMAND_START, 1, self.addr_)
        data = Receive(self.s_)
        super().updateData( data.flatten() )
        super().updateplot()




if __name__ == '__main__':
    m = OnRobotPlotter(sampleinterval=0.05, timewindow=10.,size=(1600, 1200), dict_data=[ ['fx', 'fy', 'fz'], ['tx', 'ty', 'tz'] ])
    m.run()
