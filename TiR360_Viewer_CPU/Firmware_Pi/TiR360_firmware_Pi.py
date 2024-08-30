#!/usr/bin/env python3
import cv2
import numpy as np
import argparse
import time
import io
import keyboard
import RPi.GPIO as GPIO
import pickle
from datetime import datetime



GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(9,GPIO.OUT)
GPIO.setup(6,GPIO.OUT)
GPIO.setup(5,GPIO.OUT)
GPIO.setup(8, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.output(9,GPIO.HIGH)
time.sleep(1)
GPIO.output(9,GPIO.LOW)

p = GPIO.PWM(5, 50) # GPIO 17 als PWM mit 50Hz
p.start(2.5) # Initialisierung
p.ChangeDutyCycle(0.0)
p1 = GPIO.PWM(6, 50) # GPIO 17 als PWM mit 50Hz
p1.start(2.5) # Initialisierung
p1.ChangeDutyCycle(0.0)

rec = 0
while True:
	while GPIO.input(8) == GPIO.HIGH:
		if GPIO.input(8) == GPIO.LOW:
			break 

	p.ChangeDutyCycle(0.0)
	time.sleep(0.5)
	p.ChangeDutyCycle(75.0)
	time.sleep(0.5)
	p.ChangeDutyCycle(0.0)

	time.sleep(7.5)
	insta360 = 1

	p1.ChangeDutyCycle(0.0)
	time.sleep(0.5)
	p1.ChangeDutyCycle(75.0)
	time.sleep(0.5)
	p1.ChangeDutyCycle(0.0)

	rec += 1

	parser = argparse.ArgumentParser()
	parser.add_argument("--device", type=int, default=0, help="Video Device number e.g. 0, use v4l2-ctl --list-devices")
	args = parser.parse_args()

	if args.device:
		dev = args.device
	else:
		dev = 0

	cap = cv2.VideoCapture('/dev/video'+str(dev), cv2.CAP_V4L)
	cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
	fps = cap.get(cv2.cv.CV_CAP_PROP_FPS)

	current_datetime = datetime.now().strftime("%Y-%m-%d %H-%M-%S") 
	fileTime_name = "/Video/Waermebild_" + current_datetime + str(rec) + ".tarr"
	file_name = "/Video/Waermebild_" + current_datetime + str(rec) + ".farr"
	if (cap.isOpened() == False):
		print("Error opening video stream or file")
	print("Start Capturing")
	with open(fileTime_name, 'wb') as times:
		with open(file_name, 'wb') as frames:
			while(cap.isOpened()):
				ret, frame = cap.read()
				
				#if ret == True:
				pickle.dump(frame,frames)
				pickle.dump(time.time(),times)
				if GPIO.input(8) == GPIO.LOW:
					GPIO.output(9,GPIO.LOW)
					print("Save Array")
					time.sleep(1)
					GPIO.output(9,GPIO.HIGH)
					frames = np.array(frames)
					print("Write Array to Disk!")

					GPIO.output(9,GPIO.LOW)
					time.sleep(1)
					GPIO.output(9,GPIO.HIGH)
					time.sleep(1)
					GPIO.output(9,GPIO.LOW)

					p1.ChangeDutyCycle(0.0)
					time.sleep(0.5)
					p1.ChangeDutyCycle(75.0)
					time.sleep(0.5)
					p1.ChangeDutyCycle(0.0)

					time.sleep(5)

					p.ChangeDutyCycle(0.0)
					time.sleep(0.5)
					p.ChangeDutyCycle(75.0)
					time.sleep(6.5)
					print("Complete!")
					break
