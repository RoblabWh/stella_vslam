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

class VideoStitcher(threading.Thread):
    def __init__(self, project_manager, webodm_manager, report_id, stitcher_calibration, input_videos, output_video):
        self.project_manager = project_manager
        self.webodm_manager = webodm_manager
        self.report_id = report_id
        self.progress = 0
        self.input_videos = input_videos
        self.output_video = output_video
        self.stitcher = stitcher = insta360_stitcher.Insta360Stitcher(stitcher_calibration,
                                              2880)
        super().__init__()

    def run(self):
        video1 = cv2.VideoCapture(self.input_videos[0])
        video2 = cv2.VideoCapture(self.input_videos[1])
        property_id = int(cv2.CAP_PROP_FRAME_COUNT)
        length = int(cv2.VideoCapture.get(video1, property_id))
        i = 0
        ret_large, frame_v180 = video1.read()
        ret_large, frame_h180 = video2.read()
        fps = video1.get(cv2.CAP_PROP_FPS)

        frame_large = self.stitcher.stitch(frame_v180, frame_h180)
        self.stitcher.camera_roll((3 / 2) * pi)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(self.output_video, fourcc, fps, (frame_large.shape[1], frame_large.shape[0]))
        while i < length:
            i += 1  # current number of frame being read
            progress = (i / length) * 100
            ret_large, frame_v180 = video1.read()
            ret_large, frame_h180 = video2.read()

            frame_large = self.stitcher.stitch(frame_h180, frame_v180)
            print("progress:", round(progress, 2), "frame number", i)

            # save video as file.avi
            out.write(frame_large)
        progress = 100
        out.release()



