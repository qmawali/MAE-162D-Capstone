# utility functions/classes
import numpy
import math
import cv2
import time
from bluepy import btle
import struct
import RPi.GPIO as GPIO

class rackStepper:
    def __init__(self, in1, in2, in3, in4):
        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4

        # careful lowering this, at some point you run into the mechanical limitation of how quick your motor can move
        self.step_sleep = 0.002

        #self.step_count = 4096 # 5.625*(1/64) per step, 4096 steps is 360°

        direction = False # True for clockwise, False for counter-clockwise

        # defining stepper motor sequence (found in documentation http://www.4tronix.co.uk/arduino/Stepper-Motors.php)
        self.step_sequence = [[1,0,0,1],
                        [1,0,0,0],
                        [1,1,0,0],
                        [0,1,0,0],
                        [0,1,1,0],
                        [0,0,1,0],
                        [0,0,1,1],
                        [0,0,0,1]]

        GPIO.setup(self.in1, GPIO.OUT )
        GPIO.setup(self.in2, GPIO.OUT )
        GPIO.setup(self.in3, GPIO.OUT )
        GPIO.setup(self.in4, GPIO.OUT )

        # initializing
        GPIO.output(self.in1, GPIO.LOW )
        GPIO.output(self.in2, GPIO.LOW )
        GPIO.output(self.in3, GPIO.LOW )
        GPIO.output(self.in4, GPIO.LOW )

        self.motor_pins = [self.in1,self.in2,self.in3,self.in4]
        self.motor_step_counter = 0

    def activate(self, dir):
        for pin in range(0, len(self.motor_pins)):
            GPIO.output( self.motor_pins[pin], self.step_sequence[self.motor_step_counter][pin] )
        if dir:
            self.motor_step_counter = (self.motor_step_counter + 1) % 8
        else:
            self.motor_step_counter = (self.motor_step_counter - 1) % 8

        time.sleep( self.step_sleep )

def getDirFromShelves(shelves):
    if shelves%2==0:
        return -1
    else:
        return 1

class pumpL298:
    def __init__(self, in1, in2, PWM):
        self.in1 = in1
        self.in2 = in2

        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(PWM, GPIO.OUT)

        GPIO.output(self.in1, True)
        GPIO.output(self.in2, False)

        self.PWM = GPIO.PWM(PWM, 2000)

    def activate(self, val, secs):
        self.PWM.start(val)
        time.sleep(secs)
        self.PWM.stop()

class ultrasonicSensor:
    def __init__(self, trigger, echo):
        self.trigger = trigger
        self.echo = echo

        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
    
    def distance(self):
        GPIO.output(self.trigger, True)

        time.sleep(0.00001)
        GPIO.output(self.trigger, False)
        
        TriggerTime = time.time()
        StartTime = time.time()
        StopTime = time.time()

        while GPIO.input(self.echo) == 0:
            StartTime = time.time()
            if StartTime - TriggerTime > 1:
                return -1

        while GPIO.input(self.echo) == 1:
            StopTime = time.time()
    
        # time difference between start and arrival, find travel dist
        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300) / 2
    
        return distance

# Controller class to compute PID control
class PIDController:
    # Constructor, sets PID Gains and integral/derivative trackers
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0
        self.oldErr = 0

    # Calculates control input, error
    def step(self, dT, ref, plant):
        err = ref - plant

        self.integral += err
        derivative = (err - self.oldErr)/dT

        control = self.kp*ref + self.ki*self.integral + self.kd*derivative
        return control, err
    
    # Handler function to prepare control step
    def handler(self, lastTime, ref, plant):
        dT = (time.time()-lastTime)
        controlIn, err = self.step(dT, ref, plant)
        newTime = time.time()
        return controlIn, err, newTime
    
    # reset integral and derivative trackers
    def reset(self):
        self.integral = 0
        self.oldErr = 0

# Camera class to handle opencv commands
class Camera:
    def __init__(self, camID, live):
        self.cap = cv2.VideoCapture(camID)
        self.detector = cv2.QRCodeDetector()
        self.live = live
        self.img = None

    def getImg(self):
        _, self.img = self.cap.read()

    def findNearestCode(self):
            
        if self.img is None:
                return 0,0,0,""
        # Get image from cam and detect QR Codes
        data, bbox, _ = self.detector.detectAndDecode(self.img)
        x = 0
        y = 0

        # Check if code found and store x/y data. Averages coordinates of four corners
        success = (bbox != None)
        if numpy.any(success):
                fx,fy = numpy.mean(bbox[0], 0)
                x=math.floor(fx)
                y=math.floor(fy)

        # show img if debugging
        if(self.live):
                if numpy.any(success):
                        self.img = cv2.circle(self.img, (x,y), radius=5, color = (0,0,255), thickness=-1)
                cv2.imshow('Live',self.img)
                        
                #print(data, bbox)
                cv2.waitKey(1)
            
        return numpy.any(success), x, y, data

# BLE connection class
class nanoBLE:
    def __init__(self, address, UUIDlist):
        print("Connecting to address")
        self.dev = btle.Peripheral(address)

        # connect to service
        self.service_uuid = btle.UUID(UUIDlist[0])
        self.service = self.dev.getServiceByUUID(self.service_uuid)

        # find and assign characteristics
        self.characteristics = self.dev.getCharacteristics()

        for char in self.characteristics:
            if char.uuid == UUIDlist[1]:
                self.drivetrainChar = char
            elif char.uuid == UUIDlist[2]:
                self.liftChar = char
            elif char.uuid == UUIDlist[3]:
                self.dirSwitchChar = char
    
    def __del__(self):
        self.moveDrivetrain(0)
        self.moveLift(0)
        self.dev.disconnect()

    def moveDrivetrain(self, val):
        PWM = int(numpy.min([abs(val), 255]))

        if val > 0:
            dir = 1
        else:
            dir = 0
            
        self.drivetrainChar.write(PWM.to_bytes(1, 'little', signed=False) + dir.to_bytes(1, 'little', signed=False), True)
    
    def moveLift(self, speed):
        self.liftChar.write(speed.to_bytes(2, 'little', signed=True), True)

    def readSwitch(self):
        switchTuple = struct.unpack('i', self.dirSwitchChar.read())
        return switchTuple[0]
    
    def writeSwitch(self):
        self.dirSwitchChar.write((0).to_bytes(4,'little'), True)
            
