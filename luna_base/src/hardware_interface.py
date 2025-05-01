#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Float64, Int32
from geometry_msgs.msg import Twist
from luna_base.msg import encoders

import serial
from serial.tools import list_ports
import time

#COMPORT = '/dev/ttyACM0'
COMPORT = '/dev/ttyUSB0'

#----------------------
# Defined System Values:
#----------------------
RC_FWD_MAX_DC = 10.0 # (%)
RC_BWD_MAX_DC = 5.0 # (%)
RC_STOP_DC = 7.5 # (%)
RC_FWD_MAX_PW_US = 2000.0 # (us)
RC_BWD_MAX_PW_US = 1000.0 # (us)
RC_STOP_PW_US =  1500.0 # (us)

#---------------------
# Default State Values
#---------------------
DEFAULT_SWITCH_CONTROL = 0 # 0: RC Control; 1: ROS Control; Else: RC Control
DEFAULT_ENCODER = 2147483647 # TODO: Find a better inital value
DEFAULT_MOTOR_VEL = 0.0

#----------------------
# Initial States
#----------------------
#Control Switch
global switch_control
#Encoders
global encoderLeftVal
global encoderRightVal
#Left Motor
global linearVel
global linearDC
global linearPW_US
#Right Motor
global angularVel
global angularDC
global angularPW_US
#Arm Tilt
#armTiltDir = 0 # -1: BWD; 0: Stop; 1: FWD; Else: Stop
global armTiltVel
global armTiltDC
global armTilt_PW_US
#Arm Extend
#armExtendDir = 0 # -1: BWD; 0: Stop; 1: FWD; Else: Stop
global armExtendVel
global armExtendDC
global armExtend_PW_US
#Drill
global drillMotorVel
global drillMotorDC
global drillMotorPW_US
#Bin
global binMotorVel
global binMotorDC
global binMotorPW_US

global dataFieldsSerRx

def getHardwareMsgToSer()->list:

    '''IMPORTANT NOTE: ORDER of Data Being transmitted to Serial Receiver. This MUST match the ORDER of Data on the Serial Receiver.
        dataFieldsTx = [(int),
                      (float), (float), (float), (float), (float), (float)]
    '''
    switch_control = DEFAULT_SWITCH_CONTROL #TODO: Replace with real values from Base Station
    linearVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station
    angularVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station
    armTiltVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station
    armExtendVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station
    drillMotorVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station
    binMotorVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station

    dataFieldsSerTx = [switch_control,
                       linearVel, angularVel, armTiltVel, armExtendVel, drillMotorVel, binMotorVel]
    
    # NOTE: Input is a blocking function. Comment it out to receive data when they are sent from the arduino with no delay.
    #dataFieldsSerTx = [input("Enter the value for the hardware (switch (0|1), linear(float), angular(float), armTilt(float), armExtend(float), drill(float), bin(float)): ")]
    
    return dataFieldsSerTx

def showSerialMsgRx(SerMsgRx):
    print(SerMsgRx)

def setupSerialPort()->serial.Serial:
    while True:
        try:
            ports = list_ports.comports()
            for p in ports:
                print(p)
            #COM_PORT = "COM" + input("Enter the COM PORT # (e.g. 14) for your Arduino: ")
            COM_PORT = COMPORT
            ser = serial.Serial(COM_PORT, 9600, timeout=1)  # TODO:Replace with your Arduino's port and baud rate
            #ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # TODO:Replace with your
            break
        except serial.SerialException as s:
            print(f"\nError: {s}\nCOM_PORT({COM_PORT}) selection was not correct. Please select another one.")
        except KeyboardInterrupt:
            print("Closing - KeyboardInterrupt")
            break

    print(f"Successful Serial Connection at {COM_PORT}")
    #https://github.com/curiores/ArduinoTutorials/blob/main/PythonProcess/serialReadSave.py
    # Toggle DTR to reset the Arduino
    ser.setDTR(False)
    time.sleep(1)
    ser.flushInput()
    ser.setDTR(True)
    
    return ser


ENCODER_TOPIC = '/baal/encoders'
CMD_VEL_TOPIC = '/baal/cmd_vel'
CMD_VEL_TILT_TOPIC = '/luna/cmd_vel_tilt'
CMD_VEL_EXTEND_TOPIC = '/luna/cmd_vel_extend'
CMD_VEL_DRILL_TOPIC = '/luna/cmd_vel_drill'
CMD_VEL_BIN_TOPIC = '/luna/cmd_vel_bin'
SWITCH_TOPIC = '/luna/switch'

class luna_hardware_interface():

    def __init__(self):

        #----------------------
        # Initial State Values
        #----------------------
        #Control Switch
        self.switch_control = DEFAULT_SWITCH_CONTROL # 0: RC Control; 1: ROS Control; Else: RC Control
        #Encoders
        self.encoderLeftVal = DEFAULT_ENCODER
        self.encoderRightVal = DEFAULT_ENCODER
        #Left Motor
        self.linearVel = DEFAULT_MOTOR_VEL
        self.linearDC = RC_STOP_DC
        self.linearPW_US = RC_STOP_PW_US
        #Right Motor
        self.angularVel = DEFAULT_MOTOR_VEL
        self.angularDC = RC_STOP_DC
        self.angularPW_US = RC_STOP_PW_US
        #Arm Tilt
        #armTiltDir = 0 # -1: BWD; 0: Stop; 1: FWD; Else: Stop
        self.armTiltVel = DEFAULT_MOTOR_VEL # Speed for tilt (float); positive: FWD; negative: BWD
        self.armTiltDC = RC_STOP_DC # % for DC for tilt
        self.armTilt_PW_US = RC_STOP_PW_US
        #Arm Extend
        #armExtendDir = 0 # -1: BWD; 0: Stop; 1: FWD; Else: Stop
        self.armExtendVel = DEFAULT_MOTOR_VEL # Speed for Extend; positive: FWD; negative: BWD
        self.armExtendDC = RC_STOP_DC # % for DC for Extend
        self.armExtend_PW_US = RC_STOP_PW_US
        #Drill
        self.drillMotorVel = DEFAULT_MOTOR_VEL
        self.drillMotorDC = RC_STOP_DC
        self.drillMotorPW_US = RC_STOP_PW_US
        #Bin
        self.binMotorVel = DEFAULT_MOTOR_VEL
        self.binMotorDC = RC_STOP_DC
        self.binMotorPW_US = RC_STOP_PW_US



        self.dataFieldsSerRx = [self.switch_control,
                        self.encoderLeftVal, self.encoderRightVal,
                        self.linearVel, self.angularVel, self.armTiltVel, self.armExtendVel, self.drillMotorVel, self.binMotorVel, 
                        self.linearDC, self.angularDC, self.armTiltDC, self.armExtendDC, self.drillMotorDC, self.binMotorDC,
                        self.linearPW_US, self.angularPW_US, self.armTilt_PW_US, self.armExtend_PW_US, self.drillMotorPW_US, self.binMotorPW_US]


        ENCODER_LEFT = 0
        ENCODER_RIGHT = 0
        self.ser = setupSerialPort()

        rospy.init_node('luna_hardware_interface', anonymous=True)
        #Publisher
        self.encoder_pub = rospy.Publisher(ENCODER_TOPIC, encoders, queue_size=10)
        #How often we publish
        self.rate = rospy.Rate(2) # 10hz TODO: Remove sleep for encoder publisher
        
        #Drive Subscriber
        rospy.Subscriber(CMD_VEL_TOPIC, Twist, self.drive_callback)
        #Tilt Subscriber
        rospy.Subscriber(CMD_VEL_TILT_TOPIC, Twist, self.tilt_callback)
        #Extend Subscriber
        rospy.Subscriber(CMD_VEL_EXTEND_TOPIC, Twist, self.extend_callback)
        #Drill Subscriber
        rospy.Subscriber(CMD_VEL_DRILL_TOPIC, Twist, self.drill_callback)
        #Bin Subscriber
        rospy.Subscriber(CMD_VEL_BIN_TOPIC, Twist, self.bin_callback)
        #Switch Subscriber
        rospy.Subscriber(SWITCH_TOPIC, Int32, self.switch_callback)    
	
    #Subscriber Callback: What we do when we get a message
    def drive_callback(self, data:Twist):
        #NOTE: Troubleshooting Logs
        #rospy.loginfo(rospy.get_caller_id() + "linearVel: %s\tangularVel: %s", data.linear.x, data.angular.z)
        self.linearVel=data.linear.x
        self.angularVel=data.angular.z
        
    def tilt_callback(self, data):
        #NOTE: Troubleshooting Logs
        #rospy.loginfo(rospy.get_caller_id() + f"armTiltVel: {data.linear.x}")
        self.armTiltVel=data.linear.x
        
    def extend_callback(self, data):
        #NOTE: Troubleshooting Logs
        #rospy.loginfo(rospy.get_caller_id() + f"armExtendVel: {data.linear.x}")
        self.armExtendVel=data.linear.x

    def drill_callback(self, data):
        #NOTE: Troubleshooting Logs
        #rospy.loginfo(rospy.get_caller_id() + f"drillMotorVel: {data.linear.x}")
        self.drillMotorVel=data.linear.x

    def bin_callback(self, data):
        #NOTE: Troubleshooting Logs
        #rospy.loginfo(rospy.get_caller_id() + f"binMotorVel: {data.linear.x}")
        self.binMotorVel=data.linear.x
        
    def switch_callback(self, data: Int32):
        #NOTE: Troubleshooting Logs
        #rospy.loginfo(rospy.get_caller_id() + "Switch Control: %s", data.data)
        self.switch_control = data.data

    '''
    dataFieldsSerTx = [switch_control,
                    linearVel, angularVel, armTiltVel, armExtendVel, drillMotorVel, binMotorVel]
    '''

    def sendMsgToSerial(self, msg:list):
        '''
        The message (msg) must be a list, (i.e, Sending "hello", msg = ["hello"]).

        ***IMPORTANT NOTE: The Serial Rx is expecting the following format: "<data>",
            where the message is a string that starts with a "<" and ends with a ">"
            ***This format must match the expected format for the Serial Rx***
            formatted_msg = "<data1, data2, ..., dataN>"
        '''
        formatted_msg = "<" + ", ".join(str(i) for i in msg) + ">"
        data = self.ser.write(formatted_msg.encode())

    def recvMsgFromSerial(self):
        try:
            # Read the line
            s_bytes = self.ser.readline()
            decoded_bytes = s_bytes.decode("utf-8").strip('\r\n')
            # print(decoded_bytes) # Troubleshooting
            
            data = decoded_bytes.split() # split the received data string into an array using a " " as the split.
            # print(data) # Troubleshooting

            #dataFieldsSerRx = [DEFAULT_ENCODER,DEFAULT_ENCODER,DEFAULT_ENCODER]
            '''
            ***BIG NOTE: This section MUST match the dataFieldsSerTx order from the Serial Transmitter for proper messages to be read!!!***
            dataFieldsTx = [switch_control ,
                            encoderLeftVal ,
                            encoderRightVal ,
                            linearVel ,
                            angularVel ,
                            armTiltVel ,
                            armExtendVel ,
                            drillMotorVel ,
                            binMotorVel, 
                            linearDC ,
                            angularDC ,
                            armTiltDC ,
                            armExtendDC ,
                            drillMotorDC ,
                            binMotorDC ,
                            linearPW_US ,
                            angularPW_US ,
                            armTilt_PW_US ,
                            armExtend_PW_US ,
                            drillMotorPW_US,
                            binMotorPW_US,
                            ]
            '''
            if len(data) > 0:
                
                #Control Switch
                self.switch_controlRx = int(data[0]) # 0: RC Control; 1: ROS Control; Else: RC Control
                #Encoders
                self.encoderLeftVal = int(data[1])
                self.encoderRightVal = int(data[2])
                #Left Motor
                self.linearVelRx = float(data[3])
                self.linearDC = float(data[9])
                self.linearPW_US = float(data[15])
                #Right Motor
                self.angularVelRx = float(data[4])
                self.angularDC = float(data[10])
                self.angularPW_US = float(data[16])
                #Arm Tilt
                #armTiltDir = 0 # -1: BWD; 0: Stop; 1: FWD; Else: Stop
                self.armTiltVelRx = float(data[5]) # Speed for tilt (float); positive: FWD; negative: BWD
                self.armTiltDC = float(data[11]) # % for DC for tilt
                self.armTilt_PW_US = float(data[17])
                #Arm Extend
                #armExtendDir = 0 # -1: BWD; 0: Stop; 1: FWD; Else: Stop
                self.armExtendVelRx = float(data[6]) # Speed for Extend; positive: FWD; negative: BWD
                self.armExtendDC = float(data[12]) # % for DC for Extend
                self.armExtend_PW_US = float(data[18])
                #Drill
                self.drillMotorVelRx = float(data[7])
                self.drillMotorDC = float(data[13])
                self.drillMotorPW_US = float(data[19])
                #Bin
                self.binMotorVelRx = float(data[8])
                self.binMotorDC = float(data[14])
                self.binMotorPW_US = float(data[20])

                self.dataFieldsSerRx = [self.switch_controlRx,
                                self.encoderLeftVal, self.encoderRightVal,
                                self.linearVelRx, self.angularVelRx, self.armTiltVelRx, self.armExtendVelRx, self.drillMotorVelRx, self.binMotorVelRx, 
                                self.linearDC, self.angularDC, self.armTiltDC, self.armExtendDC, self.drillMotorDC, self.binMotorDC,
                                self.linearPW_US, self.angularPW_US, self.armTilt_PW_US, self.armExtend_PW_US, self.drillMotorPW_US, self.binMotorPW_US]

                rospy.loginfo(self.dataFieldsSerRx)
        except Exception as e:
            print("Error encountered, line was not recorded.")
            print(f"{e}")
        except ValueError as a:
            print("Error encountered, line was not recorded.")
            print(f"{a}")



    def mainloop(self):
        #Publisher Loop: What we do everytime the publisher publishes
        while not rospy.is_shutdown():
            self.recvMsgFromSerial()
            encoderLeftVal = self.dataFieldsSerRx[1]
            encoderRightVal = self.dataFieldsSerRx[2]
            hardwareMsgToSer = [self.switch_control, self.linearVel, self.angularVel, self.armTiltVel, self.armExtendVel, self.drillMotorVel, self.binMotorVel]
            #getHardwareMsgToSer()
            self.sendMsgToSerial(hardwareMsgToSer)
            #ENCODER_LEFT = ENCODER_LEFT + 1
            #ENCODER_RIGHT = ENCODER_RIGHT - 1
            msg = encoders()
            msg.encoder_left = encoderLeftVal
            msg.encoder_right = encoderRightVal
            #msg = "hello world %s" % rospy.get_time()
            '''NOTE: Troubleshooting Logs
            rospy.loginfo(msg)
            rospy.loginfo(rospy.get_caller_id() + f"Switch Control: {self.switch_control}")
            rospy.loginfo(rospy.get_caller_id() + f"linearVel: {self.linearVel}\tangularVel: {self.angularVel}")
            '''
            self.encoder_pub.publish(msg)
            #self.rate.sleep()

if __name__ == '__main__':
    try:
        lhi = luna_hardware_interface()
        lhi.mainloop()
    except rospy.ROSInterruptException:
        lhi.ser.close()
        pass
