import robohat, time
import XboxController
import RPi.GPIO as gpio
import math

import sys
import tty
import termios
import subprocess


class InitioXboxControl:
	def __init__(self):
		print("Start")
		
		robohat.init()
		
		self.xValue = 0.0
		self.yValue = 0.0
		
		self.lastL = robohat.irLeft()
		self.lastR = robohat.irRight()
		
		self.servo_tilt = 22
		self.servo_pan = 18
		gpio.setup(self.servo_tilt, gpio.OUT)
		gpio.setup(self.servo_pan, gpio.OUT)
		self.tilt_pwm = gpio.PWM(self.servo_tilt, 200)
		self.pan_pwm  = gpio.PWM(self.servo_pan, 200)
		self.tilt_pwm.start(16) # 16-40
		self.pan_pwm.start(14)  # 14-20
		time.sleep(0.5) 

		self.mode = 1 # 1-AUTONOMOUS 2-Joystick
		
		self.xboxCont = XboxController.XboxController( joystickNo = 0,
    					 		       scale = 1,
    					  		       invertYAxis = False)

		self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.LTHUMBX, self.leftThumbX)
		self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.LTHUMBY, self.leftThumbY)
		self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.BACK,    self.backButton)
		self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.BACK,    self.backButton)
		self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.RB,      self.rb)
		self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.LB,      self.lb)
		
		self.xboxCont.start()
		
		self.running = True
		
	
	def rb(self, value):
		self.mode = 1
		
	def lb(self, value):
		robohat.stop()
		self.mode = 2
	
		
	def leftThumbX(self, value):
		self.xValue = value
		if self.mode == 2:
			self.updateMotors()

	def leftThumbY(self, value):
		self.yValue = value
		if self.mode == 2:
			self.updateMotors()
		
		
	def updateMotors(self):
		Y_axis = self.yValue
		X_axis = self.xValue
		r_speed, l_speed = self.joystickToDiff(X_axis, -Y_axis, -1, 1, -100, 100)
		#print("Left: " + str(l_speed) + ", Right: " + str(r_speed))
		if Y_axis < 0:
			if X_axis > 0:
				if r_speed < 0:
					r_speed = 20.0
				else:
					r_speed = r_speed + 20	
					r_speed = min(100, r_speed)
				robohat.turnForward(l_speed, r_speed)
			elif X_axis < 0:
				if l_speed < 0:
					l_speed = 20.0
				else:
					l_speed = l_speed + 20	
					l_speed = min(100, l_speed)
				robohat.turnForward(l_speed, r_speed)
			else:
				robohat.turnForward(l_speed, r_speed)		
		elif Y_axis > 0:
			if X_axis > 0:
				r_speed = -r_speed
				if l_speed < 0:
					l_speed = -l_speed + 20
					l_speed = min(100, l_speed)
				else:
					l_speed = 20.0
				robohat.turnReverse(l_speed, r_speed)
			elif X_axis < 0:
				l_speed = -l_speed
				if r_speed < 0:
					r_speed = -r_speed + 20
					r_speed = max(20, r_speed)
				else:
					r_speed = 20.0
				robohat.turnReverse(l_speed, r_speed)
			else:
				r_speed = -r_speed
				l_speed = -l_speed
				robohat.turnReverse(l_speed, r_speed)	
		else:
			if X_axis > 0:
				pass
				robohat.spinRight(abs(l_speed))
			elif X_axis < 0:
				pass
				robohat.spinLeft(abs(r_speed))
			else:
				robohat.stop()
				
				
	def autonomous(self):
		robohat.turnForward(20, 20)
		self.pan_and_tilt()
				
				
	def pan_and_tilt(self):		
		for i in range(16, 40, 2):
			self.tilt_pwm.ChangeDutyCycle(i)
			for j in range(14, 21, 2):
				self.pan_pwm.ChangeDutyCycle(j)
				self.detect_object()
				if self.mode == 2:
					break
				time.sleep(0.2)
			if self.mode == 2:
				break
				
		if (self.mode == 1):
			for i in range(40, 16, -2):
				self.tilt_pwm.ChangeDutyCycle(i)
				for j in range(14, 21, 2):
					self.pan_pwm.ChangeDutyCycle(j)
					self.detect_object()
					if self.mode == 2:
						break
					time.sleep(0.2)
				if self.mode == 2:
					break
	
		
	def backButton(self, value):
		self.xboxCont.stop()
		self.running = False
		subprocess.call(["shutdown", "-h", "now"])
		


	def map(self, v, in_min, in_max, out_min, out_max):
		# Check that the value is at least in_min
		if v < in_min:
			v = in_min
		# Check that the value is at most in_max
		if v > in_max:
			v = in_max
		return (v - in_min) * (out_max - out_min) // (in_max - in_min) + out_min



	def joystickToDiff(self, x, y, minJoystick, maxJoystick, minSpeed, maxSpeed):	

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
		rightOut = self.map(rawRight, minJoystick, maxJoystick, minSpeed, maxSpeed)
		leftOut  = self.map(rawLeft , minJoystick, maxJoystick, minSpeed, maxSpeed)

		return (rightOut, leftOut)
	

	def detect_object(self):
		newL = robohat.irLeft()
		newR = robohat.irRight()
		if (newL != self.lastL) or (newR != self.lastR):
			robohat.turnReverse(40, 40)
			time.sleep(0.7)
			robohat.spinRight(70)
			time.sleep(0.4)
			robohat.turnForward(20, 20)
		else:
			pass
			
			
if __name__ == '__main__':

	try:
		#create class
		initioCont = InitioXboxControl()
		while initioCont.running:
			if initioCont.mode == 1:
				initioCont.autonomous()
			time.sleep(0.1)

    #Ctrl C
	except KeyboardInterrupt:
		print("User cancelled")

    #Error
	except:
		print("Unexpected error:", sys.exc_info()[0])
		raise
    
	finally:
		robohat.cleanup()
    
    
