#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.

# My 2019 license: use it as much as you want. Crediting is recommended because it lets me know that I am being useful.
# Credit to Screaming Chickens 3997

# This is meant to be used in conjuction with WPILib Raspberry Pi image: https://github.com/wpilibsuite/FRCVision-pi-gen
# ----------------------------------------------------------------------------

import json
import time
import sys
from threading import Thread
import random

from cscore import CameraServer, VideoSource
from networktables import NetworkTablesInstance
import cv2
import numpy as np
from networktables import NetworkTables
from networktables.util import ntproperty
import math

########### SET RESOLUTION TO 256x144 !!!! ############

# import the necessary packages
import datetime


# Class to examine Frames per second of camera stream. Currently not used.
class FPS:
    def __init__(self):
        # store the start time, end time, and total number of frames
        # that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0

    def start(self):
        # start the timer
        self._start = datetime.datetime.now()
        return self

    def stop(self):
        # stop the timer
        self._end = datetime.datetime.now()

    def update(self):
        # increment the total number of frames examined during the
        # start and end intervals
        self._numFrames += 1

    def elapsed(self):
        # return the total number of seconds between the start and
        # end interval
        if self._end != None:
            return datetime.datetime.now() - self._start
        else:
            return datetime.datetime.now() - self._start

    def fps(self):
        # compute the (approximate) frames per second
        return self._numFrames / self.elapsed()


# class that runs separate thread for showing video,
class VideoShow:
    """
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, imgWidth, imgHeight, cameraServer, frame=None):
        self.outputStream = cameraServer.putVideo("2706_out", imgWidth, imgHeight)
        self.frame = frame
        self.stopped = False

    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            self.outputStream.putFrame(self.frame)

    def stop(self):
        self.stopped = True

    def notifyError(self, error):
        self.outputStream.notifyError(error)


# Class that runs a separate thread for reading  camera server also controlling exposure.
class WebcamVideoStream:
    def __init__(self, camera, cameraServer, frameWidth, frameHeight, name="WebcamVideoStream"):
        # initialize the video camera stream and read the first frame
        # from the stream

        # Automatically sets exposure to 0 to track tape
        self.webcam = camera
        self.webcam.setExposureManual(35)
        self.webcam.setExposureAuto()

        # Some booleans so that we don't keep setting exposure over and over to the same value

        self.autoExpose = True
        self.prevValue = True
        # Make a blank image to write on
        self.img = np.zeros(shape=(frameWidth, frameHeight, 3), dtype=np.uint8)
        # Gets the video
        self.stream = cameraServer.getVideo()
        (self.timestamp, self.img) = self.stream.grabFrame(self.img)

        # initialize the thread name
        self.name = name

        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread

            if self.stopped:
                return


            self.webcam.setExposureAuto()

            # gets the image and timestamp from cameraserver
            (self.timestamp, self.img) = self.stream.grabFrame(self.img)

    def read(self):
        # return the frame most recently read
        return self.timestamp, self.img

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def getError(self):
        return self.stream.getError()


###################### PROCESSING OPENCV ################################

# counts frames for writing images
frameStop = 0
ImageCounter = 0

# Angles in radians

# image size ratioed to 16:9
image_width = 416
image_height = 240

# Lifecam 3000 from datasheet
# Datasheet: https://dl2jx7zfbtwvr.cloudfront.net/specsheets/WEBC1010.pdf
diagonalView = math.radians(68.5)

# 16:9 aspect ratio
horizontalAspect = 16
verticalAspect = 9

# Reasons for using diagonal aspect is to calculate horizontal field of view.
diagonalAspect = math.hypot(horizontalAspect, verticalAspect)
# Calculations: http://vrguy.blogspot.com/2013/04/converting-diagonal-field-of-view-and.html
horizontalView = math.atan(math.tan(diagonalView / 2) * (horizontalAspect / diagonalAspect)) * 2
verticalView = math.atan(math.tan(diagonalView / 2) * (verticalAspect / diagonalAspect)) * 2

# Focal Length calculations: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_165
H_FOCAL_LENGTH = image_width / (2 * math.tan((horizontalView / 2)))
V_FOCAL_LENGTH = image_height / (2 * math.tan((verticalView / 2)))
# blurs have to be odd
green_blur = 1
orange_blur = 27
yellow_blur = 27

# define range of green of retroreflective tape in HSV
lower_green = np.array([40, 75, 75])
upper_green = np.array([96, 255, 255])
# define range of orange from cargo ball in HSV
lower_orange = np.array([0, 193, 92])
upper_orange = np.array([23, 255, 255])

lower_yellow = np.array([36, 50, 80])
upper_yellow = np.array([55, 120, 120])

switch = 1


# Flip image if camera mounted upside down


#################### FRC VISION PI Image Specific #############
configFile = "/boot/frc.json"


class CameraConfig: pass


team = 2706
server = False
cameraConfigs = []

"""Report parse error."""


def parseError(str):
    print("config error in '" + configFile + "': " + str, file=sys.stderr)


"""Read single camera configuration."""


def readCameraConfig(config):
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    cam.config = config

    cameraConfigs.append(cam)
    return True


"""Read configuration file."""


def readConfig():
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    return True


"""Start running the camera."""


def startCamera(config):
    print("Starting camera '{}' on {}".format(config.name, config.path))
    cs = CameraServer.getInstance()
    camera = cs.startAutomaticCapture(name=config.name, path=config.path)

    camera.setConfigJson(json.dumps(config.config))

    return cs, camera


start, switched, prevCam = True, False, 0

currentCam = 0


def switchCam():
    global currentCam, webcam, cameras, streams, cameraServer, cap, image_width, image_height, prevCam
    if networkTable.getNumber("Cam", 1):
        currentCam = 1
    else:
        currentCam = 0
    prevCam = currentCam
    cap.stop()
    webcam = cameras[currentCam]
    cameraServer = streams[currentCam]
    # Start thread reading camera
    cap = WebcamVideoStream(webcam, cameraServer, image_width, image_height).start()


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]
    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    # Name of network table - this is how it communicates with robot. IMPORTANT
    networkTable = NetworkTables.getTable('ChickenVision')

    networkTableMatch = NetworkTables.getTable("FMSInfo")

    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)

    # start cameras
    cameras = []
    streams = []
    for cameraConfig in cameraConfigs:
        cs, cameraCapture = startCamera(cameraConfig)
        streams.append(cs)
        cameras.append(cameraCapture)
    # Get the first camera

    webcam = cameras[currentCam]
    cameraServer = streams[currentCam]
    # Start thread reading camera
    cap = WebcamVideoStream(webcam, cameraServer, image_width, image_height).start()
    # cap = cap.findTape
    # (optional) Setup a CvSource. This will send images back to the Dashboard
    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(image_height, image_width, 3), dtype=np.uint8)
    # Start thread outputing stream
    streamViewer = VideoShow(image_width, image_height, cameraServer, frame=img).start()

    # cap.autoExpose=True;
    tape = True
    fps = FPS().start()
    # TOTAL_FRAMES = 200;
    # loop forever


    matchNumberDefault = random.randint(1, 1000)

    processed = 0

    while True:
        if networkTable.getBoolean("TopCamera", False):
            currentCam = 1
        else:
            currentCam = 0

        if networkTable.getNumber("Cam", currentCam) != prevCam:
            switchCam()

        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        timestamp, img = cap.read()
        if frameStop == 0:
            matchNumber = networkTableMatch.getNumber("MatchNumber", 0)
            if matchNumber == 0:
                matchNumber = matchNumberDefault
            cv2.imwrite('/mnt/VisionImages/visionImg-' + str(matchNumber) + "-" + str(ImageCounter) + '_Raw.png',
                        img)
        # Uncomment if camera is mounted upside down
        if networkTable.getBoolean("TopCamera", False):
            frame = flipImage(img)
        else:
            frame = img
        # Comment out if camera is mounted upside down
        # img = findCargo(frame,img)

        if timestamp == 0:
            # Send the output the error.
            streamViewer.notifyError(cap.getError())
            # skip the rest of the current iteration
            continue
        # Checks if you just want camera for driver (No processing), False by default

        if networkTable.getBoolean("Aligned", False):
            cv2.putText(frame, "ALIGNED", (40, 40), cv2.FONT_HERSHEY_COMPLEX, .8,
                        (255, 0, 0))

        cv2.putText(frame, "Time: " + str(fps.elapsed()), (40, 140), cv2.FONT_HERSHEY_COMPLEX, .5,
                    (255, 255, 255))

        processed = frame


        if (networkTable.getBoolean("WriteImages", True)):
            frameStop = frameStop + 1
            if frameStop == 15:
                matchNumber = networkTableMatch.getNumber("MatchNumber", 0)
                if matchNumber == 0:
                    matchNumber = matchNumberDefault
                cv2.imwrite('/mnt/VisionImages/visionImg-' + str(matchNumber) + "-" + str(ImageCounter) + '_Proc.png',
                            processed)
                frameStop = 0
                ImageCounter = ImageCounter + 1
                if (ImageCounter == 10000):
                    ImageCounter = 0

        # networkTable.putBoolean("Driver", True)
        streamViewer.frame = processed
        # update the FPS counter
        fps.update()
        # Flushes camera values to reduce latency
        ntinst.flush()
    # Doesn't do anything at the moment. You can easily get this working by indenting these three lines
    # and setting while loop to: while fps._numFrames < TOTAL_FRAMES
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
