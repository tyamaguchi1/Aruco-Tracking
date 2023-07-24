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

#############
# Arguments #
#############
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--camera", required=True, help="Set to True if using webcam")
parser.add_argument("-v", "--video", help="Path to the video file")
parser.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
parser.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
parser.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
parser.add_argument("-n", "--Point_Num", type=int, default=1, help="Number of markers to return")
args = vars(parser.parse_args())

PointNum = 1
if args["Point_Num"] != None:
    PointNum = args["Point_Num"]


LineColors = ['r-', 'g-', 'b-', '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

#Call: python Tracking_Tilt_New.py --type DICT_5X5_100 --camera True --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy -n 3

##################
# Plot functions #
##################

#Create figure 1 to plot angle
anglefig = plt.figure() #fig1
angleax = anglefig.add_subplot(1, 1, 1) #ax1

#Create figure 2 to plot distance
distancefig = plt.figure() #fig2
distanceax = distancefig.add_subplot(1, 1, 1) #ax2

#Create figure 3 to plot x-distance
ydistfig = plt.figure() #fig3
ydistax = ydistfig.add_subplot(1, 1, 1) #ax3

#Define date/time for file naming
dateStr = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")

#Initialize lists for plotting
ts = []         #initialize array for elapsed time
xf = 0          #set initial elapsed time equal to zero

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Subplot 1 (Radians)
angleax.clear()    #clear plot 

#z-components
rx = [[] for i in range(PointNum)]                            
newR = [0 for i in range(PointNum)]                            
lineR = [angleax.plot(ts,rx[i], LineColors[i])[0] for i in range(PointNum)]
print(lineR)

#Plot parameters
angleax.set_title('Radians vs. Time')
angleax.set_xlabel('Time (s)')
angleax.set_ylabel('Radians')
angleax.set_ylim(-4,4)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Subplot 2 (Z Position) 
distanceax.clear()

#markers
z_list = [[] for i in range(PointNum)]
newZ = [0 for i in range(PointNum)]
maxZ = [0 for i in range(PointNum)]
linez = [distanceax.plot(ts,z_list[i], LineColors[i])[0] for i in range(PointNum)]

#Plot parameters
distanceax.set_title('Z Position vs. Time')
distanceax.set_xlabel('Time (s)')
distanceax.set_ylabel('Z Position (mm)')

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Subplot 3 (X Position)
ydistax.clear()

#markers
x_list = [[] for i in range(PointNum)]
newX = [0 for i in range(PointNum)]
maxX = [0 for i in range(PointNum)]
linex = [ydistax.plot(ts,x_list[i],LineColors[i])[0] for i in range(PointNum)]

#plot parameters
ydistax.set_title('X Position vs. Time')
ydistax.set_xlabel('Time (s)')
ydistax.set_ylabel('X Position (mm)')
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

########################
# Global plot commands #
########################
plt.ion()
plt.show()

####################
# Video parameters #
####################
if args["camera"].lower() == "true":
    video = cv2.VideoCapture(1) + cv2.CAP_DSHOW)  #USB camera
    #video = cv2.VideoCapture(0 + cv2.CAP_DSHOW)   #webcam
    video.set(cv2.CAP_PROP_FPS, 50)
    #video.set(cv2.CAP_PROP_GAIN, 14)
    #video.set(cv2.CAP_PROP_EXPOSURE, -5)
    width = video.get(3)    # float for width of frame in mm
    height = video.get(4)   # float for height of frame in mm
else:
    if args["video"] is None:
        sys.exit(1)
    video = cv2.VideoCapture(args["video"])
if ARUCO_DICT.get(args["type"], None) is None:
    sys.exit(1)

arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters()

#Access calibration files
calibration_matrix_path = args["K_Matrix"]
distortion_coefficients_path = args["D_Coeff"]
k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)

# Initialize time 
pf = 0  # time to process previous frame
nf = 0  # time to process current frame
sf = time.time() #current time

##############################################
# Identify marker closest to center of frame #
##############################################
def detect_tilt(corners, ids, matrix_coefficients, distortion_coefficients):
    
    marker_list = []
    tilt_list = []  

    if ids is not None and len(ids) > 0:
        for i, id in enumerate(ids):
            if id is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                parameters = cv2.aruco.DetectorParameters()
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=parameters)
                rvec, tvec, rejected = cv2.aruco.estimatePoseSingleMarkers(corners, 15.0, matrix_coefficients, distortion_coefficients)
                rvec_int = rvec[0][0][2]
                tilt_list.append((i, rvec_int))

        # Sort by tilt
        sorted_markers = sorted(tilt_list, key=lambda x: x[1])

        # Return IDs of three markers if available
        for i in range(min(PointNum, len(sorted_markers))):
            marker_list.append(ids[sorted_markers[i][0]][0])
        print(marker_list)

    return marker_list

################################################################
# Calculate translation/rotation vectors for identified marker #
################################################################
def pose(frame, arucoDict, matrix_coefficients, distortion_coefficients):
    global newZ
    newZ = [0 for i in range(PointNum)]
    global newX
    newX = [0 for i in range(PointNum)]
    global newR
    newR = [0 for i in range(PointNum)]
    global maxZ
    maxZ = [0 for i in range(PointNum)]
    global maxX
    maxX = [0 for i in range(PointNum)]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=parameters)
    #If markers are detected

    if len(corners) > 0 and ids is not None:
        tilt_id = detect_tilt(corners, ids, matrix_coefficients, distortion_coefficients)
        if len(tilt_id) == PointNum and ids is not None and tilt_id is not None:
            for x in range(PointNum):
                i = np.where(ids == tilt_id[x])[0][0]
                # Estimate pose of each marker and return rvec and tvec
                rvec, tvec, rejected = cv2.aruco.estimatePoseSingleMarkers(corners[i], 15.0, matrix_coefficients, distortion_coefficients)

                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(frame, corners)

                # Draw 3D axis
                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 25)

                # tvec is x/y/z coordinate system - units: mm
                tvec_adj = tvec[0][0]

                # rvec is rotation of marker - units: rad
                rvec_adj = abs(rvec[0][0])

                #rvec 3x3 matrix
                rvec_mat = cv2.Rodrigues(rvec_adj)
                print('rvec_mat1', rvec_mat) #TODO

                #Define plot variables
                if tvec_adj is not None:
                    newZ[x] = tvec_adj[2]
                    if newZ[x] > maxZ[x]:
                        maxZ[x] = newZ[x]
                    newX[x] = abs(tvec_adj[0])
                    if newX[x] > maxX[x]:
                        maxX[x] = newX[x]
                if rvec_adj is not None:
                    newR[x] = rvec_adj[2]
    return frame

########
# Main #
########
while True:
    ret, frame = video.read()   #reads video input, outputs frame

    if ret is False:
        break

    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    # Get real-time fps
    nf = time.time()
    fps = 1 / (nf - pf)  # fps calculation
    pf = nf
    fps = int(fps)
    fps1 = str(fps)
    cv2.putText(frame, fps1, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 3, cv2.LINE_AA)
    xf = nf - sf

    #Display marker ID
    if ids is None:
        pass
    else:
        for i, marker_id in enumerate(ids):
            marker_id_str = str(marker_id[0])
            degree = str(f"deg: {marker_id_str}")
            # Check if corners exist for detected marker
            if corners[i] is not None:
                cv2.putText(frame, degree, (int(corners[i][0][0][0]), int(corners[i][0][0][1])),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    detected_markers = aruco_display(corners, ids, rejected, frame)
    output = pose(frame, arucoDict, k, d) 
    ts.append(xf) 

    #Plot 1 
    for i in range(PointNum):
        rx[i].append(newR[i])
        lineR[i].set_data(ts,rx[i])
        z_list[i].append(newZ[i])
        linez[i].set_data(ts,z_list[i])
        x_list[i].append(newX[i])
        linex[i].set_data(ts,x_list[i])
    
    angleax.set_xlim((0,(xf+10)))
    distanceax.set_ylim((-10,max(350,max(maxZ)+10)))
    distanceax.set_xlim((0,(xf+10)))
    ydistax.set_ylim(-200,200)
    ydistax.set_xlim((0,(xf+10)))

    #Call plot    
    plt.pause(.001)

    cv2.imshow('Estimated Pose', output)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
video.release()

# fig1.savefig("RPlot-"+dateStr+".png")
# fig2.savefig("ZPlot-"+dateStr+".png")
# fig3.savefig("XPlot-"+dateStr+".png")
