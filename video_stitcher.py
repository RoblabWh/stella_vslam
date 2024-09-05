#!/usr/bin/env python3

import numpy as np
import time
import datetime
import cv2
import insta360_stitcher
from math import pi
import os
import threading

class VideoStitcher(threading.Thread):
    def __init__(self, report_id, stitcher_calibration, input_videos, output_video, callback):
        self.report_id = report_id
        self.progress = 0
        self.callback = callback
        self.input_videos = input_videos
        self.output_video = output_video
        self.stitcher_calibration = stitcher_calibration
        print(os.path.exists(self.stitcher_calibration), flush=True)
        self.stitcher = insta360_stitcher.Insta360Stitcher(self.stitcher_calibration,2880)
        super().__init__()

    def run(self):
        print("started video stitcher", self.report_id, flush=True)
        video1 = cv2.VideoCapture(self.input_videos[0])
        video2 = cv2.VideoCapture(self.input_videos[1])
        property_id = int(cv2.CAP_PROP_FRAME_COUNT)
        number_of_frames = int(cv2.VideoCapture.get(video1, property_id))
        current_frame = 0
        ret_large, frame_v180 = video1.read()
        ret_large, frame_h180 = video2.read()
        fps = video1.get(cv2.CAP_PROP_FPS)

        frame_large = self.stitcher.stitch(frame_v180, frame_h180)
        self.stitcher.camera_roll((3 / 2) * pi)
        # stitcher.stitching_depth = 30
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(self.output_video, fourcc, fps, (frame_large.shape[1], frame_large.shape[0]))
        
        current_frame += 1
        
        while current_frame < (number_of_frames - 1): #or check if video1.read was true

            self.progress = (current_frame / (number_of_frames - 1)) * 100
            ret_large, frame_v180 = video1.read()
            ret_large, frame_h180 = video2.read()

            frame_large = self.stitcher.stitch(frame_h180, frame_v180)
            current_frame += 1

            # save video as file.avi
            out.write(frame_large)
        progress = 100
        out.release()
        self.callback(self.report_id)

    def get_progress(self):
        return self.progress
        
    def get_output_video(self):
        return self.output_video



