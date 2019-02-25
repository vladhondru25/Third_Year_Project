import robohat, time
import XboxController
import RPi.GPIO as gpio
import math

import sys
import tty
import termios
import subprocess

print("Start")

robohat.init()

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
	

lastL = robohat.irLeft()
lastR = robohat.irRight()

def detect_object():
	global lastL, lastR
	newL = robohat.irLeft()
	newR = robohat.irRight()
	if (newL != lastL) or (newR != lastR):
		robohat.turnReverse(40, 40)
		time.sleep(1)
		robohat.spinRight(70)
		time.sleep(0.4)
		robohat.turnForward(40, 40)
	else:
		pass

xboxCont = XboxController.XboxController( controllerCallBack = None,
    					  joystickNo = 0,
    					  deadzone = 0.3,
    					  scale = 1,
    					  invertYAxis = False)

xboxCont.start()

servo_tilt = 22
servo_pan = 18
gpio.setup(servo_tilt, gpio.OUT)
gpio.setup(servo_pan, gpio.OUT)
tilt_pwm = gpio.PWM(servo_tilt, 200)
pan_pwm = gpio.PWM(servo_pan, 200)
tilt_pwm.start(16) # 16-40
pan_pwm.start(14)  # 14-20
time.sleep(0.5) 

mode = 1 # 1-AUTONOMOUS 2-Joystick

time.sleep(1)

# main loop
try:
	while True:
		if mode == 2:
			if xboxCont.BACK:
				subprocess.call(["shutdown", "-h", "now"])
			
			if xboxCont.RB:
				mode = 1
				
			#here
			Y_axis = xboxCont.LTHUMBY
			X_axis = xboxCont.LTHUMBX
			r_speed, l_speed = joystickToDiff(X_axis, -Y_axis, -1, 1, -100, 100)
			#print("Left: " + str(l_speed) + ", Right: " + str(r_speed))
			if Y_axis < 0:
				if X_axis > 0:
					r_speed = r_speed + 50
					robohat.turnForward(l_speed, r_speed)
				elif X_axis < 0:
					l_speed = l_speed + 50
					robohat.turnForward(l_speed, r_speed)
				else:
					robohat.turnForward(l_speed, r_speed)	
			elif Y_axis > 0:
				if X_axis > 0:
					r_speed = -r_speed
					l_speed = l_speed + 50
					robohat.turnReverse(l_speed, r_speed)
				elif X_axis < 0:
					l_speed = -l_speed
					r_speed = r_speed + 50
					robohat.turnReverse(l_speed, r_speed)
				else:
					r_speed = -r_speed
					l_speed = -l_speed
					robohat.turnReverse(l_speed, r_speed)	
			else:
				if X_axis > 0:
					robohat.spinRight(abs(l_speed))
				elif X_axis < 0:
					robohat.spinLeft(abs(r_speed))
				else:
					robohat.stop()
			time.sleep(0.2)
		else:
			if xboxCont.LB:
				mode = 2
				
			robohat.turnForward(40, 40)
			
			for i in range(16, 40, 2):
				tilt_pwm.ChangeDutyCycle(i)
				for j in range(14, 21, 2):
					pan_pwm.ChangeDutyCycle(j)
					detect_object()
					time.sleep(0.2)
				if xboxCont.LB:
					mode = 2
					break
				
			if (mode == 1):
				for i in range(40, 16, -2):
					tilt_pwm.ChangeDutyCycle(i)
					for j in range(14, 21, 2):
						pan_pwm.ChangeDutyCycle(j)
						detect_object()
						time.sleep(0.2)
					if xboxCont.LB:
						mode = 2
						break
		

except KeyboardInterrupt:
    print

finally:
    robohat.cleanup()
    
