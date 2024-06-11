from utilities import *

# Main Botanist Script

# TUNING CONSTANTS
SEARCH_PWM = 50 # drivetrain PWM to run at while searching
SEARCH_LIFT_SPEED = 350 # lift speed to search with
CAMERA_X = 550 # x res to target qr code
CAMERA_Y = 258 # y res to target qr code
CODE_LOST = 4 # number of continuous frames where previously identified code is not found before giving up
SHELVES = 2 # number of shelves
LIFT_TIME_PER_SHELF = 2
LIFT_ERROR_TARGET = 20
PATH_ERROR_TARGET = 20
CM_FROM_POT = 3.5

GPIO.setmode(GPIO.BCM)

# CV object
cam = Camera(0, True)

# X/Y Controllers for Robot Positioning
path = PIDController(0,0,0)
lift = PIDController(0,0,0)


# ultrasonic sensor
ultrason = ultrasonicSensor(14, 15)

# pump
pump = pumpL298(26, 19, 13)

# rack
rack = rackStepper(17,18,27,22)

# BLE object
belinda = nanoBLE("f8:41:82:f0:f5:44", ["1214", "9873", "7758", "e94a"])
belinda.writeSwitch()
print("Initialization Complete")

calibrating = False

try:
    while calibrating:
        cam.getImg()
        _,_,_,data = cam.findNearestCode()
        print(data)

    
    # Main Loop
    while not calibrating:
        shelvesPassed = 0 # keep track of shelves checked.
        codesDone = [] 

        while (shelvesPassed<SHELVES): # while end of line has not been reached
            cam.getImg()
            codeFound, x, y, _ = cam.findNearestCode()

            if(codeFound):
                print("Code detected")
                
                lostCount = 0 # counter for number of times QR code is lost in a row, to break out
                pathCnt = liftCnt = 0 # error tracker to verify positioning success
                pathTime = liftTime = 0 # to track time for dT calculation
                pathComplete = False
                dataIDd = False
                knownData = None
                codeLost = False
                
                while liftCnt < 10:
                    cam.getImg()
                    
                    if not pathComplete:
                        belinda.moveDrivetrain(0)
                    else:
                        belinda.moveLift(0)
                    
                    contSucc, x, y, data = cam.findNearestCode()
                    
                    if not dataIDd and data != "":
                        for i in codesDone:
                            if int(data) is i:
                                print("Duplicate Code, ignore")
                                break
                            knownData = int(data)
                        dataIDd = True
                    
                    # if dataIDd:
                        # break

                    if contSucc:
                        lostCount = 0
                        # if not pathComplete:
                            # pathIn, pathErr, pathTime = path.handler(pathTime, CAMERA_X, x) 
                            # print("Moving Drivetrain: " + str(pathIn))
                            # belinda.moveDrivetrain(int(pathIn))
                            # if pathErr < PATH_ERROR_TARGET:
                                # pathCnt += 1
                                # if pathCnt > 5:
                                    # belinda.moveDrivetrain(0)
                                    # print("Drivetrain in Position")
                                    # pathComplete = True
                        # else:
                            # liftIn, liftErr, liftTime = lift.handler(liftTime, CAMERA_Y, y) 
                            # print("Moving Lift" + str(liftIn))
                            # belinda.moveLift(int(liftIn))
                            # if liftErr < LIFT_ERROR_TARGET:
                                # liftCnt += 1
                        if not pathComplete:
                            if abs(CAMERA_X - x) < PATH_ERROR_TARGET:
                                pathCnt += 1
                                if pathCnt > 3:
                                    pathComplete = True
                                    print("Drivetrain in Position")
                            elif CAMERA_X - x > 0:
                                print("Moving Drivetrain")
                                belinda.moveDrivetrain(SEARCH_PWM)
                            else:
                                print("Moving Drivetrain")
                                belinda.moveDrivetrain(-SEARCH_PWM)
                        else:
                            if abs(CAMERA_Y - y) < LIFT_ERROR_TARGET:
                                liftCnt += 1
                            elif CAMERA_Y - y > 0:
                                print("Moving Lift")
                                belinda.moveLift(SEARCH_LIFT_SPEED)
                            else:
                                print("Moving Lift")
                                belinda.moveLift(-SEARCH_LIFT_SPEED)
                    else:
                        lostCount += 1
                        print("Lost Code: " + str(lostCount))
                        if lostCount>CODE_LOST:
                            print("Lost COde!!!")
                            belinda.moveDrivetrain(0)
                            belinda.moveLift(0)
                            codeLost = True
                            break
                
                if dataIDd:
                    continue
                
                if not codeLost:
                    belinda.moveLift(0)
                    print("Lift in Position")

                    while knownData is None:
                        print("Code unreadable!!!")
                        cam.getImg()
                        _,_,_,data = cam.findNearestCode()
                        if data != "":
                            knownData = int(data)
                    
                    print("Forward Rack")
                    # watering
                    
                    distCalCount = 15
                    
                    initialDist = 0
                    valuesCount = 0
                    while valuesCount < distCalCount:
                        dist = ultrason.distance()
                        if dist == -1:
                            continue
                        print("Initial Dist Calibration: " + str(dist))
                        if dist < 50:
                            initialDist += dist
                            valuesCount += 1
                    
                    initialDist /= distCalCount
                    print("Initial Dist: " + str(initialDist))
                    
                    while dist > CM_FROM_POT:
                        valuesCount = 0
                        dist = ultrason.distance()
                        if dist == -1:
                            continue
                        print("Forward Rack: " + str(dist))
                        while valuesCount < distCalCount:
                            dist = ultrason.distance()
                            if dist == -1:
                                continue
                            if dist < CM_FROM_POT:
                                valuesCount += 1
                            else:
                                dist = 50
                                break
                        rack.activate(1)

                    print("Reached Pot: Watering")
                    
                    time.sleep(0.5)
                    pump.activate(50, knownData+5)
                    time.sleep(0.5)
                    
                    
                    dist = initialDist-.1
                    while dist <= initialDist-.1:
                        valuesCount = 0
                        dist = ultrason.distance()
                        if dist == -1:
                            continue
                        print("Retracting Rack: " + str(dist))
                        while valuesCount < distCalCount:
                            dist = ultrason.distance()
                            if dist == -1:
                                continue
                            if dist > initialDist -.1:
                                valuesCount += 1
                            else:
                                dist = 0
                                break
                        rack.activate(0)
                        
                    codesDone.append(knownData)

            #end of line reached, move up and switch
            if belinda.readSwitch():
                print("switch")
                belinda.moveDrivetrain(0)
                time.sleep(1)
                belinda.writeSwitch()
                
                shelvesPassed += 1
                if shelvesPassed < SHELVES:
                    print("Shelves: " + str(shelvesPassed))
                    belinda.moveLift(SEARCH_LIFT_SPEED)
                    time.sleep(LIFT_TIME_PER_SHELF)
                    belinda.moveLift(0)

            else:
                print("Searching")
                belinda.moveDrivetrain(getDirFromShelves(shelvesPassed) * SEARCH_PWM)

        
        print("Sleeping")
        belinda.moveLift(-SEARCH_LIFT_SPEED)
        time.sleep(SHELVES*LIFT_TIME_PER_SHELF)
        belinda.moveLift(0)
        print("Lift Retracted")
        time.sleep(30)
        print("Relooping")
finally:
    GPIO.cleanup()
