import json
import os
import subprocess
import socketio
import eventlet
import map_segment_pb2
import base64
import cv2
from util import popen_and_call
from VSlamProcess import vslam_thread
from video_stitcher import VideoStitcher
from flask import Flask, flash, request, redirect, url_for, render_template, jsonify

class SlamServer:
    def __init__(self, address, port):
        self.app = Flask(__name__)
        self.app.secret_key = "AchtzichEurodaisterPruegelaberjutbezahlt"
        self.address = address
        self.port = port
        self.slam_statuses = {}
        self.stitcher_statuses = {}
        self.threads = []
        self.results = []
        self.updates_threads = []

    def setup_routes(self):
        self.app.add_url_rule('/slam', methods=['POST'], view_func=self.start_slam)
        self.app.add_url_rule('/sticher', methods=['POST'], view_func=self.start_stitcher)
        self.app.add_url_rule('/get_sticher_status', methods=['GET'], view_func=self.stitcher_status)
        self.app.add_url_rule('/get_slam_status', methods=['GET'], view_func=self.slam_status)
        self.app.add_url_rule('/remove_thread/<report_id>', methods=['GET'], view_func=self.remove_thread)
        self.app.add_url_rule('/get_slam_map', methods=['GET'], view_func=self.get_slam_maps)

    def start_slam(self):
        data = request.get_json(force=True)
        report_id = data['report_id']
        video = data['video']
        config = data['config']
        mask = data['mask']
        orb_vocab = data['orb_vocab']
        slam_options = data['slam_options']
        other_options = data['other_options']
        update_interval = other_options['update_frequency']
        keyfrm_path = data['keyfrm_path']
        landmark_path = data['landmark_path']
        keyfrm_folder_path = data['keyfrm_folder_path']
        map_db_output = data['map_db_output']
        slam_output_path = data['slam_output_path']
        args = '/stella_vslam_examples/build/run_video_slam -m ' + str(video) + ' -v ' + str(orb_vocab) + ' -c ' + str(config)
        if(mask is not None):
            args = args + ' --mask ' + str(mask)
        if int(slam_options['frame_skip']) > 1 :
            args = args + ' --frame-skip ' + str(slam_options['frame_skip'])
        if slam_options['no_sleep'] :
            args = args + ' --no-sleep '
        args = args +  ' --auto-term -k ' + str(keyfrm_folder_path) + ' -o ' + str(map_db_output)
        print(args.split(), flush=True)
        process = vslam_thread(report_id, keyfrm_path, landmark_path, slam_output_path, update_interval, self.updateCallback) #update interval in seconds
        self.threads.append(process)
        process.start()
        self.slam_statuses[str(report_id)] = 'started'
        result = popen_and_call(on_exit=self.slam_finished, report_id=report_id, popen_args=args.split())  # add output for map db and stuff
        return jsonify({'slam': 'started'})

    def slam_finished(self, report_id):
        if (str(report_id) in self.slam_statuses) and (self.slam_statuses[str(report_id)] == 'started'):
            self.slam_statuses[str(report_id)] = 'finished'

    def map_update(self, report_id, data):
        print("map_update")

    def updateCallback(self, report_id):
        self.updates_threads.append(report_id)

    def slam_status(self):
        thread_data = []
        print(self.updates_threads, flush=True)
        for thread in self.threads:
            if (str(thread.report_id) in self.slam_statuses) and (self.slam_statuses[str(thread.report_id)] == 'started'):
                done = False
            if (str(thread.report_id) in self.slam_statuses) and (self.slam_statuses[str(thread.report_id)] == 'finished'):
                done = True
            if (thread.report_id in self.updates_threads):
                self.updates_threads.remove(thread.report_id)
                update = True
            else:
                update = False
            thread_data.append({'report_id': thread.report_id, 'started': thread.report_id, 'progress': thread.progress,  'update': update, 'done': done})
        return jsonify(thread_data)

    def get_slam_maps(self):
        return jsonify(self.results)

    def remove_thread(self, report_id):
        removed = False
        for i, thread in enumerate(self.threads):
            if thread.report_id == report_id:
                thread.stop()
                thread.join()
                break

        self.threads.pop(i)
        removed = True

        return jsonify({'removed': removed})

    def start_stitcher(self):
        data = request.get_json(force=True)
        report_id = data['report_id']
        stitcher_calibration = data['stitcher_calibration']
        input_videos = data['input_videos']
        output_video = data['output_video']
        thread = VideoStitcher(report_id, stitcher_calibration, input_videos, output_video, self.stitcher_finished)
        self.threads.append(thread)
        thread.start()
        self.stitcher_statuses[str(report_id)] = 'started'
        return jsonify("true")

    def stitcher_status(self, report_id):
        thread_data = []
        print(self.updates_threads, flush=True)
        for thread in self.threads:
            if (str(thread.report_id) in self.stitcher_statuses) and (
                    self.stitcher_statuses[str(thread.report_id)] == 'started'):
                done = False
            if (str(thread.report_id) in self.slam_statuses) and (
                    self.stitcher_statuses[str(thread.report_id)] == 'finished'):
                done = True
            if (thread.report_id in self.updates_threads):
                self.updates_threads.remove(thread.report_id)
                update = True
            else:
                update = False
            thread_data.append({'report_id': thread.report_id, 'started': thread.report_id, 'progress': thread.progress,
                                'update': update, 'done': done})
        return jsonify(thread_data)

    def stitcher_finished(self, report_id):
        if (str(report_id) in self.stitcher_statuses) and (self.stitcher_statuses[str(report_id)] == 'started'):
            self.stitcher_statuses[str(report_id)] = 'finished'

    def run(self):
        self.app.run(host=self.address, port=self.port, debug=False, use_reloader=False)
