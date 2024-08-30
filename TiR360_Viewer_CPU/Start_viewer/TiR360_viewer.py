#!/usr/bin/env python3

# import imutils
import numpy as np
# import argparse
import time
# import io
import pickle
import datetime
import cv2
import insta360_stitcher
import cv2 as cv
from math import pi
# from imutils.video import FPS
import tkinter as tk
from tkinter import filedialog
import glob
import os
import threading

# from PyQt6.QtCore import QObject, QThread, pyqtSignal, Qt, QTimer
# from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QApplication
# from PyQt6.QtGui import QImage, QPixmap

# Stitcher
morph = 0
depth = 100
stitcher = insta360_stitcher.Insta360Stitcher(os.path.join(os.path.dirname(__file__), "../config/calibration.json"),
                                              2880)

# 256x192 General settings
width = 256  # Sensor width
height = 192  # sensor height
scale = 3.5  # scale multiplier
newWidth = int(width * scale)
newHeight = int(height * scale)
alpha = 0  # Contrast control (1.0-3.0)
colormap = 0
font = cv2.FONT_HERSHEY_SIMPLEX
dispFullscreen = False
rad = 0  # blur radius
threshold = 2
hud = False
recording = False
elapsed = "00:00:00"
snaptime = "None"
xt = 0
yt = 0


def convert_to_timestamp(entry):
    year = entry[0]
    day_of_year = entry[1]

    # Erstellen Sie ein datetime-Objekt mit dem Jahr und dem Tag des Jahres
    dt = datetime.datetime(year=year, month=1, day=1) + datetime.timedelta(days=day_of_year - 1)
    return dt


def overlay_frames(frame_large, frame_small, position):
    global y_kor
    global x_kor
    global alpha
    global y_offset
    global x_offset
    y_offset, x_offset = position
    y1, y2 = y_offset, y_offset + frame_small.shape[0]
    x1, x2 = x_offset, x_offset + frame_small.shape[1]

    alphai = alpha / 255.0  # frame_small[:, :, 2] / 255.0
    for c in range(0, 3):
        frame_large[y1:y2, x1:x2, c] = (1 - alphai) * frame_large[y1:y2, x1:x2, c] + alphai * frame_small[:, :, c]
    # frame_large[y1:y2, x1:x2] = frame_small
    return frame_large


def showTemp(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        # print('({}, {})'.format(x, y))

        # imgCopy = heatmap.copy()
        # cv2.circle(imgCopy, (x, y), 10, (255, 0, 0), -1)
        xt = x
        yt = y

    cv2.putText(heatmap, str(tempm) + ' C', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(heatmap, str(tempm) + ' C', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    # cv2.imshow('Thermal', heatmap)


def rec():
    now = time.strftime("%Y%m%d--%H%M%S")
    # do NOT use mp4 here, it is flakey!
    videoOut = cv2.VideoWriter(now + 'output.avi', cv2.VideoWriter_fourcc(*'XVID'), 25, (newWidth, newHeight))
    return (videoOut)


def snapshot(heatmap):
    # I would put colons in here, but it Win throws a fit if you try and open them!
    now = time.strftime("%Y%m%d-%H%M%S")
    snaptime = time.strftime("%H:%M:%S")
    cv2.imwrite("TC001" + now + ".png", heatmap)
    return snaptime


def button_insert_file(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Datei eingefügt")


def button_download(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Download gestartet")


def valmap(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def findindexfortime(arr, atime, delta):
    index = 0
    global TCamCalib
    for t in arr:
        index = index + 1

        if t - delta >= atime:
            break

    if ((t - delta) - atime > 0.1):
        TCamCalib = 1
    else:
        TCamCalib = 0

    return index - 1


def update_trackbar():
    global i
    global title_window
    global trackbar_name

    while True:
        with position_lock:
            cv2.setTrackbarPos(trackbar_name, title_window, i)
        # cv2.waitKey(1000)  # Kurze Pause, um CPU-Last zu reduzieren


def fstop():
    global stop
    if stop == 1:
        stop = 0
    else:
        if stop == 0:
            stop = 1


position_lock = threading.Lock()

# Ordner Dialog
folder = filedialog.askdirectory()
file1 = glob.glob(folder + "/*_00_*.insv")
file2 = glob.glob(folder + "/*_10_*.insv")
print("Read video array")
thermalImage = False
# frames = np.load('/home/jan/tc001.npy')

# Read 360Grad

video1 = cv2.VideoCapture(file1[0])
video2 = cv2.VideoCapture(file2[0])
property_id = int(cv2.CAP_PROP_FRAME_COUNT)
length = int(cv2.VideoCapture.get(video1, property_id))
print("stitching video with ", length, " frames")
# cv2.CAP_PROP_POS_MSEC
print("Read complete!")

# Länge der gelesenen Frames
i = 0

ret_large, frame_v180 = video1.read()
ret_large, frame_h180 = video2.read()

fps = video1.get(cv2.CAP_PROP_FPS)

frame_large = stitcher.stitch(frame_v180, frame_h180)
stitcher.camera_roll((3 / 2) * pi)
# stitcher.stitching_depth = 30
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output360Thermal.avi', fourcc, fps, (frame_large.shape[1], frame_large.shape[0]))

# Wenn sich die ThermalCamera Recalibriert wird der wert auf 1 gesetzt
TCamCalib = 0
# threading.Thread(target=update_trackbar, daemon=True).start()
progress = 0
while i < length:

    i += 1 #current number of frame being read
    progress = i / length
    ret_large, frame_v180 = video1.read()
    ret_large, frame_h180 = video2.read()

    frame_large = stitcher.stitch(frame_h180, frame_v180)
    print("progress:", round(progress, 2), "frame number", i)

    # save video as file.avi
    out.write(frame_large)



