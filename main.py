import robohat, time
import XboxController
import RPi.GPIO as gpio
import math

import sys
import tty
import termios
import subprocess

from mvnc import mvncapi as mvnc
from imutils.video import VideoStream
import imutils
import argparse
import numpy as np
import time
import cv2

CLASSES = ("background", "aeroplane", "bicycle", "bird",
	"boat", "bottle", "bus", "car", "cat", "chair", "cow",
	"diningtable", "dog", "horse", "motorbike", "person",
	"pottedplant", "sheep", "sofa", "train", "tvmonitor")
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# frame dimensions should be sqaure
PREPROCESS_DIMS = (300, 300)
DISPLAY_DIMS = (900, 900)

# calculate the multiplier needed to scale the bounding boxes
DISP_MULTIPLIER = DISPLAY_DIMS[0] // PREPROCESS_DIMS[0]

def preprocess_image(input_image):
	# preprocess the image
	preprocessed = cv2.resize(input_image, PREPROCESS_DIMS)
	preprocessed = preprocessed - 127.5
	preprocessed = preprocessed * 0.007843
	preprocessed = preprocessed.astype(np.float16)

	# return the image to the calling function
	return preprocessed

def predict(image, graph):
	# preprocess the image
	image = preprocess_image(image)

	# send the image to the NCS and run a forward pass to grab the
	# network predictions
	graph.LoadTensor(image, None)
	(output, _) = graph.GetResult()

	# grab the number of valid object predictions from the output,
	# then initialize the list of predictions
	num_valid_boxes = output[0]
	predictions = []

	# loop over results
	for box_index in range(int(num_valid_boxes)):
		# calculate the base index into our array so we can extract
		# bounding box information
		base_index = 7 + box_index * 7

		# boxes with non-finite (inf, nan, etc) numbers must be ignored
		if (not np.isfinite(output[base_index]) or
			not np.isfinite(output[base_index + 1]) or
			not np.isfinite(output[base_index + 2]) or
			not np.isfinite(output[base_index + 3]) or
			not np.isfinite(output[base_index + 4]) or
			not np.isfinite(output[base_index + 5]) or
			not np.isfinite(output[base_index + 6])):
			continue

		# extract the image width and height and clip the boxes to the
		# image size in case network returns boxes outside of the image
		# boundaries
		(h, w) = image.shape[:2]
		x1 = max(0, int(output[base_index + 3] * w))
		y1 = max(0, int(output[base_index + 4] * h))
		x2 = min(w,	int(output[base_index + 5] * w))
		y2 = min(h,	int(output[base_index + 6] * h))

		# grab the prediction class label, confidence (i.e., probability),
		# and bounding box (x, y)-coordinates
		pred_class = int(output[base_index + 1])
		pred_conf = output[base_index + 2]
		pred_boxpts = ((x1, y1), (x2, y2))

		# create prediciton tuple and append the prediction to the
		# predictions list
		prediction = (pred_class, pred_conf, pred_boxpts)
		predictions.append(prediction)

	# return the list of predictions to the calling function
	return predictions

def camera_funct(vs, obj_to_detect):
	# grab the frame from the threaded video stream
	# make a copy of the frame and resize it for display/video purposes
	frame2 = vs.read()
	frame = imutils.rotate(frame2, angle=180)
	image_for_result = frame.copy()
	image_for_result = cv2.resize(image_for_result, DISPLAY_DIMS)
	#image_for_result = imutils.rotate(image_for_result, 180)

	# use the NCS to acquire predictions
	predictions = predict(frame, graph)

	# loop over our predictions
	for (i, pred) in enumerate(predictions):
		# extract prediction data for readability
		(pred_class, pred_conf, pred_boxpts) = pred
		if   pred_class == 9  and obj_to_detect == "chair":
			pass
		elif pred_class == 11 and obj_to_detect == "table":
			pass
		elif pred_class == 15 and obj_to_detect == "face":
			pass
		elif pred_class == 20 and obj_to_detect == "laptop":
			pass
		else:
			continue
		# filter out weak detections by ensuring the `confidence`
		# is greater than the minimum confidence
		if pred_conf > 0.5:
			# build a label consisting of the predicted class and
			# associated probability
			label = "{}: {:.2f}%".format(CLASSES[pred_class],
				pred_conf * 100)
			# extract information from the prediction boxpoints
			(ptA, ptB) = (pred_boxpts[0], pred_boxpts[1])
			ptA = (ptA[0] * DISP_MULTIPLIER, ptA[1] * DISP_MULTIPLIER)
			ptB = (ptB[0] * DISP_MULTIPLIER, ptB[1] * DISP_MULTIPLIER)
			(startX, startY) = (ptA[0], ptA[1])
			y = startY - 15 if startY - 15 > 15 else startY + 15
				
			# display the rectangle and label text
			cv2.rectangle(image_for_result, ptA, ptB, (255,0,0), 2)
			cv2.putText(image_for_result, label, (startX, y),
				cv2.FONT_HERSHEY_SIMPLEX, 1, COLORS[pred_class], 3)

	# display the frame on the screen with prediction data 
	# (you can achieve faster FPS if you do not output to the screen)
	# !!! MODIFY BELOW !!!
	if True:
		# display the frame to the screen
		cv2.imshow("Output", image_for_result)
		key = cv2.waitKey(1) & 0xFF


class InitioXboxControl:
	def __init__(self):
		print("Start")
		
		robohat.init()
		
		self.obj_to_detect = 'None'
		
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
		self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.Y,       self.buttY)
		self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.X,       self.buttX)
		self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.A,       self.buttA)
		self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.B,       self.buttB)
		
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
			
	def buttY(self, value):
		self.obj_to_detect = 'face'
	def buttX(self, value):
		self.obj_to_detect = 'chair'
	def buttA(self, value):
		self.obj_to_detect = 'table'
	def buttB(self, value):
		self.obj_to_detect = 'laptop'
		
		
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
				
				
	def autonomous(self,vs):
		robohat.turnForward(20, 20)
		self.pan_and_tilt(vs)
				
				
	def pan_and_tilt(self,vs):		
		for i in range(16, 40, 2):
			self.tilt_pwm.ChangeDutyCycle(i)
			for j in range(14, 21, 2):
				self.pan_pwm.ChangeDutyCycle(j)
				self.detect_object()
				camera_funct(vs, self.obj_to_detect)
				if self.mode == 2:
					break
				time.sleep(0.1)
			if self.mode == 2:
				break
				
		if (self.mode == 1):
			for i in range(40, 16, -2):
				self.tilt_pwm.ChangeDutyCycle(i)
				for j in range(14, 21, 2):
					self.pan_pwm.ChangeDutyCycle(j)
					self.detect_object()
					camera_funct(vs, self.obj_to_detect)
					if self.mode == 2:
						break
					time.sleep(0.1)
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
		# grab a list of all NCS devices plugged in to USB
		devices = mvnc.EnumerateDevices()
		device = mvnc.Device(devices[0])
		device.OpenDevice()
		
		# open the CNN graph file
		with open(".//Robot//graphs//mobilenetgraph", mode="rb") as f:
			graph_in_memory = f.read()
		
		# load the graph into the NCS
		graph = device.AllocateGraph(graph_in_memory)
		
		# open a pointer to the video stream thread and allow the buffer to
		# start to fill
		vs = VideoStream(usePiCamera=True).start()
		time.sleep(1)
		
		#create class
		initioCont = InitioXboxControl()
		while initioCont.running:
			if initioCont.mode == 1:
				initioCont.autonomous(vs)
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
		
		# stop the video stream
		vs.stop()

		# clean up the graph and device
		graph.DeallocateGraph()
		device.CloseDevice()
    
    
