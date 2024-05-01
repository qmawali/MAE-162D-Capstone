# utility functions/classes
import numpy
import cv2
import time

# handler function for both lift, path motors
def controlHandler(lastTime, controller, ref, out):
    dT = (time.perf_counter_ns() - lastTime)/(10e9)
    controlIn, err = controller.step(dT, ref, out)
    newTime = time.perf_counter_ns()
    return controlIn, err, newTime

def controlErrorHandler(err, cnt):
    if(err<10):
        return cnt+1
    else:
        return 0

# Controller class to compute PID control
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0
        self.oldErr = 0

    def step(self, dT, ref, plantOut):
        err = ref - plantOut

        integral += err
        derivative = (err - self.oldErr)/dT

        control = self.kp*ref + self.ki*integral + self.kd*derivative
        return control, err
    
    def reset(self):
        self.integral = 0
        self.oldErr = 0

# Camera class to handle opencv commands
class Camera:
    def __init__(self, camID, live):
        self.cap = cv2.VideoCapture(camID)
        self.detector = cv2.QRCodeDetector()
        self.live = live

    def findNearestCode(self):
        _, img = self.cap.read()
        data, bbox, _ = self.detector.detectAndDecode(img)

        success = (bbox != None)
        x = numpy.mean(bbox[0], 0)
        y = numpy.mean(bbox[0], 1)

        if(self.live):
            cv2.imshow(img)
            print(data, bbox)

        return success, x, y
