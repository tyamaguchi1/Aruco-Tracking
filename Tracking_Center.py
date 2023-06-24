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

#Call: python Tracking_Center.py --type DICT_5X5_100 --camera True --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy

##################
# Plot functions #
##################

#Create figure 1 to plot angle
fig1 = plt.figure()
ax1 = fig1.add_subplot(1, 1, 1)

#Create figure 2 to plot distance
fig2 = plt.figure()
ax2 = fig2.add_subplot(1, 1, 1)

#Define date/time for file naming
dateStr = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")

#Initialize lists for plotting
ts = []         #initialize array for elapsed time
xf = 0          #set initial elapsed time equal to zero

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Subplot 1 (Radians)
ax1.clear()    #clear plot 

#z-component1
rx1 = []                             
newR1 = 0                            
lineR1, = ax1.plot(ts,rx1, 'r-') 

#z-component2
rx2 = []                             
newR2 = 0                            
lineR2, = ax1.plot(ts,rx2, 'g-')

#z-component3 
rx3 = []                             
newR3 = 0                            
lineR3, = ax1.plot(ts,rx3, 'b-')   

#Plot parameters
ax1.set_title('Radians vs. Time')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Radians')
ax1.set_ylim(-4,4)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Subplot 2 (Z Position) 
ax2.clear()    #clear plot

#closest marker
z_list1 = []     #initialize array for z distances
newz1 = 0        
maxz1 = 0        
linez1, = ax2.plot(ts, z_list1, 'r-')

#second marker
z_list2 = []
newz2 = 0
maxz2 = 0
linez2, = ax2.plot(ts, z_list2, 'b-')

#third marker
z_list3 = []
newz3 = 0
maxz3 = 0
linez3, = ax2.plot(ts, z_list3, 'g-')

#plot parameters
ax2.set_title('Z Position vs. Time')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Z Position (mm)')
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


########################
# Global plot commands #
########################
plt.ion()
plt.show()


#############
# Arguments #
#############
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--camera", required=True, help="Set to True if using webcam")
parser.add_argument("-v", "--video", help="Path to the video file")
parser.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
parser.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
parser.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
args = vars(parser.parse_args())


####################
# Video parameters #
####################
if args["camera"].lower() == "true":
    #video = cv2.VideoCapture(1 + cv2.CAP_DSHOW)  #USB camera
    video = cv2.VideoCapture(0 + cv2.CAP_DSHOW)   #webcam
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
def detect_three_closest_markers(frame, corners, ids):
    # Calculate center of the frame
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2

    closest_ids = []

    if ids is not None and len(ids) > 0:
        # Calculate distance from each marker to center of the frame
        distances = []
        for i, corner in enumerate(corners):
            if corner is not None:
                x = int(np.mean(corner[:, 0]))
                y = int(np.mean(corner[:, 1]))
                distance = np.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
                distances.append((i, distance))

        # Sort by distance
        sorted_markers = sorted(distances, key=lambda x: x[1])

        # Return IDs of three closest markers if available
        for i in range(min(3, len(sorted_markers))):
            closest_ids.append(ids[sorted_markers[i][0]][0])
        print(closest_ids)

    return closest_ids


################################################################
# Calculate translation/rotation vectors for identified marker #
################################################################
def pose(frame, arucoDict, matrix_coefficients, distortion_coefficients):
    global newz1
    global newz2
    global newz3
    global newR1
    global newR2
    global newR3
    global maxz1
    global maxz2
    global maxz3

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=parameters)

    # If markers are detected
    if len(corners) > 0:
        if ids is not None:
            closest_id = detect_three_closest_markers(frame, corners, ids)
            if len(closest_id) == 3:
                if ids is not None:
                    if closest_id is not None:
                        i = np.where(ids == closest_id[0])[0][0]
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
                        print('rvec_mat1', rvec_mat)

                        #Define plot variables
                        if tvec_adj is not None:
                            newz1 = tvec_adj[2]
                            if newz1 > maxz1:
                                maxz1 = newz1
                        if rvec_adj is not None:
                            newR1 = rvec_adj[2]


                if ids is not None:
                    closest_id = detect_three_closest_markers(frame, corners, ids)
                    if closest_id is not None:
                        i = np.where(ids == closest_id[1])[0][0]
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
                        print('rvec_mat2', rvec_mat)

                        #Define plot variables
                        if tvec_adj is not None:
                            newz2 = tvec_adj[2]
                            if newz2 > maxz2:
                                maxz2 = newz2
                        if rvec_adj is not None:
                            newR2 = rvec_adj[2]

                if ids is not None:
                    closest_id = detect_three_closest_markers(frame, corners, ids)
                    if closest_id is not None:
                        i = np.where(ids == closest_id[2])[0][0]
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
                        print('rvec_mat3', rvec_mat)

                        #Define plot variables
                        if tvec_adj is not None:
                            newz3 = tvec_adj[2]
                            if newz3 > maxz3:
                                maxz3 = newz3
                        if rvec_adj is not None:
                            newR3 = rvec_adj[2]

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
    rx1.append(newR1)
    rx2.append(newR2)
    rx3.append(newR3)
    lineR1.set_data(ts,rx1)
    lineR2.set_data(ts,rx2)
    lineR3.set_data(ts,rx3)
    ax1.set_xlim((0,(xf+10)))
    

    #Plot 2
    z_list1.append(newz1)
    z_list2.append(newz2)
    z_list3.append(newz3) 
    linez1.set_data(ts,z_list1)
    linez2.set_data(ts,z_list2)
    linez3.set_data(ts,z_list3)
    MAX = max(maxz1, maxz2, maxz3)
    ax2.set_ylim((-10,max(350,MAX+10)))
    ax2.set_xlim((0,(xf+10)))

    #Call plot    
    plt.pause(.001)

    cv2.imshow('Estimated Pose', output)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
video.release()

#fig1.savefig("RPlot-"+dateStr+".png")
#fig2.savefig("ZPlot-"+dateStr+".png")