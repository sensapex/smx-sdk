"""
SensapexUnit.py

Brendan Callahan, Alex Chubykin
The Picower Institute for Learning and Memory Massachusetts Institute of Technology
e-mail: chubykin(at)mit.edu

A simple class which can ask the micromanipulation system for coordinates, move to a specific location, and ask if the motors are moving or not.  

LICENCE FOR SensapexUnit.py

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""


from serial import Win32Serial
import io
import sys
import time
import copy
import crc16my2 as crc

class SensapexUnit(Win32Serial):
    
    ASCII_Commands={'STX':0x02,'DLE':0x10,'NAK':0x15,'ETX':0x03,'ESC':0x1b,'ACK':0x06}
    ERRORS = {0:"UMANIPULATORCTL_NO_ERROR", -1: "UMANIPULATORCTL_OS_ERROR", -2:"UMANIPULATORCTL_NOT_OPEN",-3:"UMANIPULATORCTL_TIMEOUT",-4:"UMANIPULATORCTL_INVALID_ARG",-5:"UMANIPULATORCTL_INVALID_DEV",-6:"UMANIPULATORCTL_INVALID_RESP",-7:"UMANIPULATORCTL_INVALID_CRC"}
    STATUS = {-1:"UMANIPULATORCTL_STATUS_READ_ERROR",0x01:"UMANIPULATORCTL_STATUS_X_MOVING",0x02:"UMANIPULATORCTL_STATUS_Y_MOVING",0x04:"UMANIPULATORCTL_STATUS_Z_MOVING",0x10:"UMANIPULATORCTL_STATUS_X_BUSY",0x20:"UMANIPULATORCTL_STATUS_Y_BUSY",0x40:"UMANIPULATORCTL_STATUS_Z_BUSY",0x80:"UMANIPULATORCTL_STATUS_JAMMED"}
    MODE = {-1:"UMANIPULATORCTL_MODE_READ_ERROR",0:"UMANIPULATORCTL_MODE_UNKNOWN",1:"UMANIPULATORCTL_MODE_1",2:"UMANIPULATORCTL_MODE_2",3:"UMANIPULATORCTL_MODE_3",4:"UMANIPULATORCTL_MODE_4",5:"UMANIPULATORCTL_MODE_5",6:"UMANIPULATORCTL_MODE_PEN",7:"UMANIPULATORCTL_MODE_SNAIL"}
    MAX_MANIPULATORS = 15
    DEF_REFRESH_TIME = 2500
    MAX_POSITION = 20400
    
    full_step=4.9705
    
    

    def __init__(self, port, baudrate, unitID, **kwargs):
        """ 
        Initialize the Serial superclass, and setup some default values
        """

        super(SensapexUnit,self).__init__(port=port,baudrate=baudrate,**kwargs)
        #self.device={'IsPresent':1,'position':0,'pitch':0}
                
        self.silent = False
        
        self.parent = None
        
        self.numdevices = 1
        self.device = 1
        self.stepLengthUM = None
        
        self.unitID = unitID
        self.unitType = "SX"
        
        self._reconfigurePort()
        self.getStepLength()
        
        self.x0EndPosPlus = False
        self.x0EndPosMinus = False
        self.y0EndPosPlus = False
        self.y0EndPosMinus = False
        self.z0EndPosPlus = False
        self.z0EndPosMinus = False
        
        self.x1EndPosPlus = False
        self.x1EndPosMinus = False
        self.y1EndPosPlus = False
        self.y1EndPosMinus = False
        self.z1EndPosPlus = False
        self.z1EndPosMinus = False
        
    def getStepLength(self):
        message = ''.join(self.talk("141f0000")[5:9])
        print message
        #Step length is returned as a multiple of 10 nanometers.  So if 6 is returned
        #by the device, that means that the step length is 60 nanometers, or .06 um
        stepnm = int(message,16)/100.0
        self.stepLengthUM = stepnm
        
    def setParent(self,parentin):
        self.parent = parentin

    #Not that this function is ever going to be used aside from testing,
    #but it should only be given integers as parameters, as the position values
    #for the device are integers.
    def move_to(self, manip, x=None, y=None, z=None):
        
        for i in range (0,3):
            message = []
            message.append(str(self.device))
            message.append("3") #move to
            if i == 0:
                message.append("16")
                hexval = (hex(x))
            elif i == 1:
                message.append("17")
                hexval = (hex(y))
            elif i == 2:
                message.append("18")
                hexval = (hex(z))
            
            hexval = hexval[2:]
            if len(hexval) < 4:
                for i in range (len(hexval),4):
                    hexval = "0"+hexval
            message.append(hexval)
        
            self.talk(''.join(message))
            
        self.talk("1000000f")
    
    def move_to_um(self, manip, x=None, y=None, z=None):
        
        x = int(round(x/self.stepLengthUM))
        y = int(round(y/self.stepLengthUM))
        z = int(round(z/self.stepLengthUM))        
        
        for i in range (0,3):
            
            message = []
            message.append(str(self.device))
            message.append("3") #move to
            if i == 0:
                message.append("18")
                hexval = (hex(x))
            elif i == 1:
                message.append("16")
                hexval = (hex(y))
            elif i == 2:
                message.append("17")
                hexval = (hex(z))
            
            hexval = hexval[2:]
            if len(hexval) < 4:
                for i in range (len(hexval),4):
                    hexval = "0"+hexval
            message.append(hexval)
        
            self.talk(''.join(message))
            
        self.talk("1000000f")
        
    def move_relative_um(self,device,x,y,z):
        
        print "relval",x
        x = x*self.stepLengthUM
        y = y*self.stepLengthUM
        z = z*self.stepLengthUM
        
        self.get_motors_coord_um()
        self.move_to_um(1,int(self.parent.motorcoordinates[self.unitID][0][0]+x),int(self.parent.motorcoordinates[self.unitID][0][1]+y),int(self.parent.motorcoordinates[self.unitID][0][2]+z))
        
    def waitUntilReady(self,manip = None):
        pass
        
        
    
        
    #has not been tested.  however it should hopefully work
    def getChecksum(self,message):
        crc = 0xFFFF
        
        for msgchar in range (0,len(message)):
            crc = ((0xFF00 & crc) | (ord(message[msgchar]) & 0x00FF) ^ (crc & 0x00FF))
            for lask in range (8,0,-1):
                if (crc & 0x0001) == 0:
                    crc >>=1
                else:
                    crc >>=1
                    crc ^= 0xA001
                    
        crcstr = hex(crc)[2:]
        crcupper = ""
        for charnum in range (0,len(crcstr)):
            crcupper += crcstr[charnum].upper()
        
        if len(crcupper) != 4:
            for i in range (len(crcupper),4):
                crcupper = "0"+crcupper
        
        return crcupper
    
    
    def serial_read(self):
        self.data_read=[]
        a=self.read()
        if a!=0:
            self.data_read.append(a)
            while a!=0:
                a=self.read()
                self.data_read.append(a)
        return self.data_read
    
    def serial_read_all(self):
        return self.readline()           
    
    def serial_write(self,data,iteration = 0):
        try:
            return self.write(data)
        except:
            print "SERIAL WRITE EXCEPTION, RESENDING DATA"
            if iteration == 20:
                print "################MAX SERIAL WRITE EXCEPTIONS REACHED, RETURNING###################"
                return
            else:
                return self.serial_write(data,iteration+1)
    
    def serial_purge(self):
        self.flushInput()
        self.flushOutput()
        pass
        
    def talk(self,toSend):
        self.serial_purge()
        checksum=self.getChecksum(toSend)
        toSend = chr(self.ASCII_Commands['STX'])+toSend+checksum+chr(self.ASCII_Commands['ETX'])
            
        ordchars = []
        
        for character in toSend:
            ordchars.append(character)
            
        print ordchars
        print hex(ord(ordchars[0]))
        charchars=''.join(ordchars)
         
        print charchars
        
        self.serial_write(charchars)
        
        
        receivedchars = []
        for i in range (0,14):
            receivedchars.append(self.read())
            
        print "RECEIVED: ",receivedchars
        device = receivedchars[1]
        command = receivedchars[2]
        address = ''.join(receivedchars[3:5])    
        data = ''.join(receivedchars[5:9])
        
        wholemessage = ''.join(receivedchars)
        
        #print device, command, address, data
        
        if command == '3':
            if address == '0A':
                print "X is ",int(data,16)
            if address == '08':
                print "Y is ",int(data,16)
            if address == '09':
                print "Z is ",int(data,16)
                
        return wholemessage
        
    
    def get_motors_coord_um(self, manip=None):
        print self.parent
        if manip == None:
            devrangestart = 1
            devrangeend = self.numdevices
        else:
            devrangestart = manip+1
            devrangeend = manip+1
        
        for dev in range (devrangestart,devrangeend+1):
            for motor in range(0,3):
                if motor == 0:
                    address = "0A"
                elif motor == 1:
                    address = "08"
                elif motor == 2:
                    address = "09"
                
                
                message = int(''.join(self.talk(str(dev)+"4"+address+"0000")[5:9]),16)*self.stepLengthUM
                print "MESSAGE: ",message
                
                if self.parent != None:
                    print "##################MESSAGE#########################"
                    print self.parent.motorcoordinates
                    self.parent.motorcoordinates[self.unitID][dev-1][motor]=copy.deepcopy(message)
            