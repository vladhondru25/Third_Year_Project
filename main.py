import robohat, time
import XboxController

import sys
import tty
import termios

print("Start")

xboxCont = XboxController.XboxController( controllerCallBack = None,
    					  joystickNo = 0,
    					  deadzone = 0.3,
    					  scale = 1,
    					  invertYAxis = False)

robohat.init()
xboxCont.start()

# main loop
try:
	while True:
		#here
		Y_axis = -xboxCont.LTHUMBY
		X_axis =  xboxCont.LTHUMBX
		
		
		
		
		
		
		
		
		
		
		
#robohat.forward(speed)
#robohat.reverse(speed)
#robohat.spinRight(speed)
#robohat.spinLeft(speed)
#robohat.stop()
            

except KeyboardInterrupt:
    print

finally:
    robohat.cleanup()
    

