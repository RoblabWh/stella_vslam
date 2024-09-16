import eventlet
import socketio
import threading
import requests
import base64
import map_segment_pb2
import pyarrow
import numpy
import datetime
import json
from keyframes import Keyframes
from landmarks import Landmarks
from transmission import Transmission
from setinterval import setInterval

#process that communicates with stellavslam
class vslam_thread(threading.Thread):

    #decodes the messages send via socketviewer from protobuf
    #saves keyframes in a keyframe structure
    #saves landmarks in a landmark structure
    def decodeMessage(self, data):
        map = map_segment_pb2.map()
        map.ParseFromString(base64.b64decode(data))
        for msg in map.messages:
            if msg.tag == 'progress':
                self.progress = float(msg.txt)
        for keyframe in map.keyframes:
            keyfrm = {}
            keyfrm["id"] = keyframe.id
            keyfrm["pose"] = []  # convert pose to array
            for cell in keyframe.pose.pose:
                keyfrm["pose"].append(cell)
            lastStep = "keyframe" + str(keyframe.id) + "|" + str(keyfrm["pose"]) + ":"
            if (keyfrm["pose"] is None):
                self.keyframes.removeKeyframe(keyfrm["id"])
            else:
                self.keyframes.updateKeyframe(keyfrm["id"], keyfrm["pose"])
        for landmark in map.landmarks:
            landmarkObj = {}
            landmarkObj["id"] = landmark.id
            #if the landmark has coordinates, we create a new landmark that will either get added or update a current
            #entry in self.landmarks
            if (len(landmark.coords) != 0):
                landmarkObj["point_pos"] = []
                for coord in landmark.coords:
                    landmarkObj["point_pos"].append(coord)
                landmarkObj["rgb"] = []
                for color in landmark.color:
                    landmarkObj["rgb"].append(color)
            if (len(landmark.coords) == 0):
                self.landmarks.removeLandmark(landmarkObj["id"])
            else:
                self.landmarks.updateLandmark(landmarkObj["id"], landmarkObj["point_pos"], landmarkObj["rgb"])
        if (self.interval is None):
            self.interval = setInterval(self.push_update_interval, self.saveResults)

    #initializes the vslam thread
    #push update interval: frequency how often the interval_callback is being called
    #interval_callback: callback being called, depending on push update interval
    def __init__(self, report_id, keyfrm_path, landmark_path, slam_output_path, push_update_interval, interval_callback):
        self.report_id = report_id
        self.sio = socketio.Server()
        self.app = socketio.WSGIApp(self.sio)
        self.keyframes = Keyframes()
        self.landmarks = Landmarks()
        self.keyfrm_path = keyfrm_path
        self.landmark_path = landmark_path
        self.slam_output_path = slam_output_path
        self.transmission = Transmission(callback=self.decodeMessage)
        self.interval_callback = interval_callback
        self.startTime = None
        self.stopTime = None
        self.done = False
        self.progress = 0
        self.push_update_interval = push_update_interval #in s
        self.interval = None
        self.newUpdate = False

        @self.sio.event
        def connect(sid, environ):
            print('connect ', sid)

        @self.sio.on('map_publish')
        def handle_message(sid, data):
            if self.startTime == None:
                self.startTime = datetime.datetime.now()
                self.lastTime_update_push = datetime.datetime.now()
            self.currentTime = datetime.datetime.now()
            self.transmission.receive(data)

        #disconnect event considers vslam as being finished
        @self.sio.event
        def disconnect(sid):
            print('disconnect ', sid)
            self.interval.cancel()
            self.stopTime = datetime.datetime.now()
            self.saveResults()
            self.saveSlamResults()
            self.done = True
        super().__init__()

    #saves keyframes and landmarks as json files in the project
    def saveResults(self):
        with open(self.keyfrm_path, "w") as json_keyfrm_file:
            json.dump(self.keyframes.getAllKeyframes(), json_keyfrm_file, ensure_ascii=False, indent=4)
        with open(self.landmark_path, "w") as json_landmark_file:
            json.dump(self.landmarks.getAllLandmarks(), json_landmark_file, ensure_ascii=False, indent=4)
        self.interval_callback(self.report_id)

    #saves vslam runtime in a json file
    def saveSlamResults(self):
        print("finished vslam in", self.stopTime - self.startTime, flush=True)
        calc_time = self.stopTime - self.startTime
        slam_output = {"calculation_time": str(calc_time)}
        with open(self.slam_output_path, "w") as json_slam_file:
            json.dump(slam_output, json_slam_file, ensure_ascii=False, indent=4)

    #opens socket to communicate with stellavslam
    def run(self):
        eventlet.wsgi.server(eventlet.listen(('', 3000)), self.app)
