from utilities import *

# Main Botanist Script

# Initializations

# CV object
cam = Camera(0, True)

# X/Y Controllers for Robot Positioning
lift = PIDController(1,0,0)
path = PIDController(1,0,0)

# Main Loop
while True:
    while (True): # TODO arduino is not finding end of line
        crntTime = time.perf_counter_ns()
        codeFound, x, y = cam.findNearestCode()
        
        # 
        if(codeFound):
            lostCount = 0 # counter for number of times QR code is lost in a row, to break out
            pathCnt = liftCnt = 0 # error tracker to verify positioning success
            pathTime = liftTime = 0 # to track time for dT calculation
            
            while(pathCnt < 10 and liftCnt < 10):
                contSucc, x, y = cam.findNearestCode()

                if(contSucc):
                    lostCount = 0

                    pathIn, pathErr, pathTime = controlHandler(pathTime, path, 310, x) 
                    # TODO send pathIn to Arduino
                    pathCnt = controlErrorHandler(pathErr, pathCnt)


                    liftIn, liftErr, liftTime = controlHandler(liftTime, lift, 240, y) 
                    # TODO send liftIn to Arduino
                    liftCnt = controlErrorHandler(liftErr, liftCnt)
                
                else:
                    lostCount += 1
                    if(lostCount>20):
                        break

        # TODO tell Arduino to slowly move forward while lift goes up/down


    # TODO tell Arduino to return to base, determine sleep time and sleep zzz
