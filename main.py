import robohat, time
import XboxController
import math

import sys
import tty
import termios

print("Start")

def map(v, in_min, in_max, out_min, out_max):
	# Check that the value is at least in_min
	if v < in_min:
		v = in_min
	# Check that the value is at most in_max
	if v > in_max:
		v = in_max
	return (v - in_min) * (out_max - out_min) // (in_max - in_min) + out_min



def joystickToDiff(x, y, minJoystick, maxJoystick, minSpeed, maxSpeed):	

# If x and y are 0, then there is not much to calculate...
	if x == 0 and y == 0:
		return (0, 0)
    

	# First Compute the angle in deg
	# First hypotenuse
	z = math.sqrt(x * x + y * y)

	# angle in radians
	rad = math.acos(math.fabs(x) / z)

	# and in degrees
	angle = rad * 180 / math.pi

	# Now angle indicates the measure of turn
	# Along a straight line, with an angle o, the turn co-efficient is same
	# this applies for angles between 0-90, with angle 0 the coeff is -1
	# with angle 45, the co-efficient is 0 and with angle 90, it is 1

	tcoeff = -1 + (angle / 90) * 2
	turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
	turn = round(turn * 100, 0) / 100

	# And max of y or x is the movement
	mov = max(math.fabs(y), math.fabs(x))

	# First and third quadrant
	if (x >= 0 and y >= 0) or (x < 0 and y < 0):
		rawLeft = mov
		rawRight = turn
	else:
		rawRight = mov
		rawLeft = turn

	# Reverse polarity
	if y < 0:
		rawLeft = 0 - rawLeft
		rawRight = 0 - rawRight

	# minJoystick, maxJoystick, minSpeed, maxSpeed
	# Map the values onto the defined rang
	rightOut = map(rawRight, minJoystick, maxJoystick, minSpeed, maxSpeed)
	leftOut = map(rawLeft, minJoystick, maxJoystick, minSpeed, maxSpeed)

	return (rightOut, leftOut)

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
		Y_axis = xboxCont.LTHUMBY
		X_axis = xboxCont.LTHUMBX
		r_speed, l_speed = joystickToDiff(X_axis, -Y_axis, -1, 1, -100, 100)
		#print("Left: " + str(l_speed) + ", Right: " + str(r_speed))
		if Y_axis < 0:
			if X_axis > 0:
				r_speed = r_speed + 50
				#robohat.turnForward(l_speed, r_speed)
			elif X_axis < 0:
				l_speed = l_speed + 50
				#robohat.turnForward(l_speed, r_speed)
			else:
				pass
				#robohat.turnForward(l_speed, r_speed)	
		elif Y_axis > 0:
			if X_axis > 0:
				r_speed = -r_speed
				l_speed = l_speed + 50
				#robohat.turnReverse(l_speed, r_speed)
			elif X_axis < 0:
				l_speed = -l_speed
				r_speed = r_speed + 50
				#robohat.turnReverse(l_speed, r_speed)
			else:
				r_speed = -r_speed
				l_speed = -l_speed
				#robohat.turnReverse(l_speed, r_speed)	
			print("Left: " + str(l_speed) + ", Right: " + str(r_speed)) 
		else:
			if X_axis > 0:
				robohat.spinRight(abs(l_speed))
			elif X_axis < 0:
				robohat.spinLeft(abs(r_speed))
			else:
				robohat.stop()
		

except KeyboardInterrupt:
    print

finally:
    robohat.cleanup()
    

