import serial
import asyncio
import threading
import os
import sys
import math
import numpy as np
import Sofa.Core
from serial import SerialException

AngleCorrection = 90

class serialReader(Sofa.Core.Controller):
    def __init__(self):
        Sofa.Core.Controller.__init__(self)
        self.name = "serialReader"
        try:
            self.ser = serial.Serial("COM4", baudrate = 115200)
        except SerialException:
            Sofa.msg_error(__file__, "Please plug the serial bus in")
            self.ser = None
        self.angleflat = [0, 0, 0, 0]
        self.angleper = [0, 0, 0, 0]
        self.t1 = threading.Thread(target = self.Read)


    def Read(self):
        while True:
            value = 0
            value = self.ser.read(6)
            leg = value[0]
            self.angleper[leg] = value[2] | (value[3])<<8
            self.angleflat[leg] = value[4] | (value[5])<<8
            if(self.angleflat[leg] > 32768):
                self.angleflat[leg] = -65535 + self.angleflat[leg]
            if(self.angleper[leg] > 32768):
                self.angleper[leg] = -65535 + self.angleper[leg]
            self.angleflat[leg] = self.angleflat[leg]/32
            self.angleper[leg] = self.angleper[leg]/32
            print(self.angleflat[0])


    def Start(self) :
        self.t1.start()
        print("started")

    def getAngles(self, leg):
        Flat = self.angleflat[leg] - AngleCorrection
        Per = self.angleper[leg] - AngleCorrection
        
        return Flat, Per