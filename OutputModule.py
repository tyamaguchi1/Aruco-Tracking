import cv2
import numpy as np
import sys
import argparse
import time
from datetime import datetime
import csv
from library import ARUCO_DICT, aruco_display
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
import math
from config import CamArray

def setup():
    cameras = [[] for x in range(len(CamArray))]

    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--type", type=str, default="DICT_5X5_100", help="Type of ArUCo tag to detect")
    args = vars(parser.parse_args())

    if ARUCO_DICT.get(args["type"], None) is None:
        sys.exit(1)

    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
    arucoParams = cv2.aruco.DetectorParameters()
    print("CamArray:",CamArray)
    for camera in CamArray:
        print(camera[1])
        video = cv2.VideoCapture(camera[1] + cv2.CAP_DSHOW)  #USB camera
        #video = cv2.VideoCapture(0 + cv2.CAP_DSHOW)   #webcam
        video.set(cv2.CAP_PROP_FPS, 50)
        video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        #video.set(cv2.CAP_PROP_GAIN, 14)
        #video.set(cv2.CAP_PROP_EXPOSURE, -5)
        cameras[camera[0]].append(video)

        k = np.load(camera[5])
        d = np.load(camera[6])
        cameras[camera[0]].append(k)
        cameras[camera[0]].append(d)
        if not video.isOpened():
            print("failed to open camera")
            sys.exit(1)
    return cameras, arucoDict


#takes an array of  4 int arrays representing the square's corners and returns the center point
def GetSquareCenterFromCorners(corners):
    center = [0,0]
    distance = 0
    corners = corners[0]
    for x in range(4):
        for y in range(4):
            current = math.sqrt((corners[x][0]-corners[y][0])**2+(corners[y][1]-corners[y][1])**2)
            if current > distance:
                distance = current
                center[0] = (corners[x][0]+corners[y][0])/2
                center[1] = (corners[x][1]+corners[y][1])/2
    return center

#takes a frame from a camera and returns the center, rotation, and dimensions of the Aruco marker of least tilt
def GetInformation(camera, arucoDict):
    global frame
    ret, frame = camera[0].read()
    frame = cv2.flip(frame, -1)   #reads video input, outputs frame
    if ret is False:
        print("Failed to get frame from camera",camera[0])
        return None
    name = "Camera" + str(camera[0])
    cv2.imshow(name, frame)
    return pose(frame, arucoDict, camera[1], camera[2])

################################################################
# Return [ID, center x offset, rotation, x frame dimension]#
################################################################
def pose(frame, arucoDict, matrix_coefficients, distortion_coefficients):
    Output = [0, 0, 0, 0]

    gray = frame# cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=parameters)
    #If markers are detected
    
    if len(corners) > 0 and ids is not None:
        tilt_id = detect_tilt(corners, ids, matrix_coefficients, distortion_coefficients, arucoDict)
        if len(tilt_id) == 1 and ids is not None and tilt_id is not None:
            
            i = np.where(ids == tilt_id[0])[0][0]
            # Estimate pose of each marker and return rvec and tvec
            rvec, tvec, rejected = cv2.aruco.estimatePoseSingleMarkers(corners[i], 15.0, matrix_coefficients, distortion_coefficients)
#! 15.0
            center = GetSquareCenterFromCorners(corners[i])
            # rvec is rotation of marker - units: rad
            rvec_adj = rvec[0][0]#abs(rvec[0][0])

            #correction
            if rvec_adj[0]<0:
                rvec_adj[2]*=-1
            if rvec_adj[2] < -0.087:
                rvec_adj[2]+= (math.pi/4)

            #Define plot variables
            if rvec_adj is not None:
                Output[2] = rvec_adj[2]
            else:
                return None
            
            Output[1] = center[0]
            Output[0] = tilt_id[0]
            return Output
    return None

##############################################
# Identify marker closest to center of frame #
##############################################
def detect_tilt(corners, ids, matrix_coefficients, distortion_coefficients, arucoDict):
    global frame
    marker_list = []
    tilt_list = []  

    if ids is not None and len(ids) > 0:
        for i, id in enumerate(ids):
            if id is not None:
                gray = frame#cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                parameters = cv2.aruco.DetectorParameters()
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=parameters)
                rvec, tvec, rejected = cv2.aruco.estimatePoseSingleMarkers(corners, 15.0, matrix_coefficients, distortion_coefficients)
                
                rvec_int = rvec[0][0][2]
                tilt_list.append((i, abs(rvec_int)))

        # Sort by tilt
        sorted_markers = sorted(tilt_list, key=lambda x: x[1])

        # Return IDs of three markers if available
        marker_list.append(ids[sorted_markers[0][0]][0])
        #print(marker_list)

    return marker_list

#Gets the expected vector to center of the maze relative to the camera's heading - returns [cam_id, angle from camera, rotation of maze, offset]
#NOTE: The id=0 marker being straight on is defined as the 0º rotation. We then assume that MARKER_NUM markers are incremented and spaced evenly around the maze
def GetRelativeVector(output, camera_id, cameras, MARKER_NUM, CAM_FOV):
    vector = [camera_id, 0.0, 0.0, 0.0]
    AngleBetweenMarkers = 2*math.pi/MARKER_NUM
    id = output[0]
    id -= output[2]/AngleBetweenMarkers
    if(id<0):
        id+=MARKER_NUM
    if(id>=MARKER_NUM):
        id-=MARKER_NUM
    vector[2] = id*AngleBetweenMarkers

    #vector[3] = math.sin(output[2])*ARENA_RADIUS
    
    width = cameras[camera_id][0].get(3)

    dist = output[1]-(width/2)
    vector[1] = -(dist/width)*(CAM_FOV*math.pi/180)
    #print(vector[1])
    return vector

#Calculates the physical vector for the maze from the relative vector data - returns the matrix [[x-off, y-off],[x-slope, y-slope]]
def GetAbsoluteVector(vector):
    outputMatrix = [[0.0,0.0],[0.0,0.0]]
    CamX, CamY, CamRot = 0,0,0
    for camera in CamArray:
        if camera[0] == vector[0]:
            CamX = camera[3]
            CamY = camera[4]
            CamRot = camera[2]
            break
    if CamX == 0 and CamY == 0 and CamRot == 0:
        return None
    xProp = math.sin(CamRot)
    yProp = math.cos(CamRot)
    outputMatrix[0][0] = CamX+(xProp*vector[3])
    outputMatrix[0][1] = CamY+(yProp*vector[3])
#    print("Cam Angle:",CamRot)
#    print("Maze Angle:",vector[1])
    TotalAngle = -(math.pi-(vector[1]+CamRot))
    outputMatrix[1][0] = math.cos(TotalAngle)
    outputMatrix[1][1] = math.sin(TotalAngle)

    return outputMatrix
    
def GetIntersection(v1, v2):
    #lines ab and cd
    a = [v1[0][0], v1[0][1]]
    b = [v1[0][0]+v1[1][0], v1[0][1]+v1[1][1]]
    c = [v2[0][0], v2[0][1]]
    d = [v2[0][0]+v2[1][0], v2[0][1]+v2[1][1]]

    term1 = [c[0]-a[0], c[1]-a[1]]
    term2 = [d[0]-a[0], d[1]-a[1]]
    term3 = [b[0]-a[0], b[1]-a[1]]

    d1 = np.cross(term1, term3)
    d2 = np.cross(term2, term3)

    term4 = d1 * d
    term5 = d2 * c
    term6 = [term4[0]-term5[0], term4[1] - term5[1]]
    return (term6) / (d1 - d2)


#Main Function - cycles through cameras, getting their packets, and then calculates and reports the center of the maze
def Snatch(cameras, arucoDict, CAM_FOV, MARKER_NUM, ARENA_RADIUS, REDUNDANCY):
#    while True:
    AvgRot = 0
    Counted = 0
    AvgX = 0
    AvgY = 0
    Intersections = []

    count = 0
    while count < REDUNDANCY:
        vectors = []
        count+=1
        for x in range(len(cameras)):
            output = GetInformation(cameras[x], arucoDict)
            if output is not None:
    #            print("output:",output)
                Counted+=1
                RelativeVector = GetRelativeVector(output, x, cameras, MARKER_NUM, CAM_FOV)
                for camera in CamArray:
                    if camera[0] == x:
                        Rot = camera[2]-RelativeVector[2]
                        if(Rot < 0):
                            Rot+=math.pi*2
                        if(Rot > 2*math.pi):
                            Rot -= math.pi*2
                        AvgRot+= Rot
                        break
                vector = GetAbsoluteVector(RelativeVector)
                print("y-",vector[0][1],"=",vector[1][1]/vector[1][0], "(x-", vector[0][0], ")")
                vectors.append(vector)
        
        for vector in vectors:
            for pair in vectors:
                if vector is not pair:
                    Intersections.append(GetIntersection(vector, pair))
    
    for intersection in Intersections:
        AvgX+=intersection[0]
        AvgY+=intersection[1]
#        print("(",intersection[0],",",intersection[1],")")
    if(Counted > 0 and len(Intersections) > 0):
        AvgX/=len(Intersections)
        AvgY/=len(Intersections)
        AvgRot/=Counted
        #print("Maze is at (", AvgX, ",", AvgY, ") with rotation ", AvgRot*180/math.pi, "º")
#        print("(", AvgX, ",", AvgY, ")")
        return (AvgX, AvgY, AvgRot, Intersections)
    return None