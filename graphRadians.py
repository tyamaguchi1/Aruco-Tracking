import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
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
from library import ARUCO_DICT, aruco_display

#python graphRadians.py --type DICT_5X5_100 --camera True --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

dateStr = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
xf = 0

ts = []

rx1 = []
newR1 = 0
ax.clear()
lineR1, = ax.plot(ts,rx1, 'r-')

rx2 = []
newR2 = 0
lineR2, = ax.plot(ts,rx2, 'g-')

rx3 = []
newR3 = 0
lineR3, = ax.plot(ts,rx3, 'b-')



# ax.set_ylim((-10,350))
# ax.set_xlim((-10,200))

#format plot
plt.xticks(rotation=90, ha='right')
plt.title('Z Position vs. Time')
plt.ylabel('Z Position')
plt.xlabel('Time')

fig.canvas.draw()

plt.ion()
plt.show()
fig.canvas.draw()

# arguments
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--camera", required=True, help="Set to True if using webcam")
parser.add_argument("-v", "--video", help="Path to the video file")
parser.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
parser.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
parser.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
args = vars(parser.parse_args())

if args["camera"].lower() == "true":
    #video = cv2.VideoCapture(1 + cv2.CAP_DSHOW)
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FPS, 5)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    video.set(cv2.CAP_PROP_GAIN, 14)
    video.set(cv2.CAP_PROP_EXPOSURE, -5)

else:
    if args["video"] is None:
        # print("[Error] Video file location is not provided")
        sys.exit(1)

    video = cv2.VideoCapture(args["video"])

if ARUCO_DICT.get(args["type"], None) is None:
    # print(f"ArUCo tag type '{args['type']}' is not supported")
    sys.exit(1)

arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters()

# Marker closest to frame
def detect_closest_marker(frame, corners, ids):
    # Calculate center of the frame
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2

    if ids is None:
        pass

    else:
        if len(ids) == 0:
            return None

        # Calculate distance from each marker to center of the frame
        distances = []
        for i, corner in enumerate(corners):
            if corner is not None:
                x = int(np.mean(corner[:, 0]))
                y = int(np.mean(corner[:, 1]))
                distance = np.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
                distances.append((i, distance))

        # Sort by distance and return ID of closest marker
        if len(distances) > 0:
            closest_marker = sorted(distances, key=lambda x: x[1])[0]
            return ids[closest_marker[0]][0]

        else:
            return None

def pose(frame, arucoDict, matrix_coefficients, distortion_coefficients):
    global newR1
    global newR2
    global newR3
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=parameters)

    # If markers are detected
    if len(corners) > 0:
        if ids is not None:
            closest_id = detect_closest_marker(frame, corners, ids)
            if closest_id is not None:
                i = np.where(ids == closest_id)[0][0]
                # Estimate pose of each marker and return rvec and tvec
                rvec, tvec, rejected = cv2.aruco.estimatePoseSingleMarkers(corners[i], 15.0, matrix_coefficients,
                                                                           distortion_coefficients)

                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(frame, corners)

                # Draw 3D axis
                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

                # tvec is x/y/z coordinate system - units: mm
                # adjust origin to center of frame
                tvec_adj = tvec[0][0]
                #print('tvec', tvec_adj)
                newR1 = rvec[0][0][0]
                newR2 = rvec[0][0][1]
                newR3 = rvec[0][0][2]
                print(newR1,newR2,newR3)
                # rvec is rotation of marker
                #rvec_array = np.array(rvec[0][0])
                # rotation_matrix, _ = cv2.Rodrigues(rvec)
                # _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(np.hstack((rotation_matrix,tvec.reshape(-1))), dtype=object)
                # rotation_y = euler_angles[1]
                # angle = np.rad2deg(rotation_y)
                # print('rvec', rvec)
                # print("sum: ", rvec[0][0][0]+rvec[0][0][1])
                #print('angle', angle)

                # plot
                if tvec_adj is not None:
                    newZ = tvec_adj[2]

                # np.save('rvec', rvec_array)
                # np.save('tvec', tvec_adj)
                # with open('z_log-'+dateStr+'.xls', mode='a', newline='\n') as f:
                #     if f.tell() == 0:
                #         writer = csv.writer(f, delimiter='\t')
                #         writer.writerow(['Z', 'Elapsed Time'])

                # #Append the IDs and elapsed time to the Excel file
                #     if f.tell() > 0:
                #         with open('z_log-'+dateStr+'.xls', mode='a', newline='\n') as f:
                #             writer = csv.writer(f, delimiter='\t')
                #             writer.writerow([((tvec_adj[2]+6.022)/1.0797), xf])
    return frame


vcap = cv2.VideoCapture(1)
width = vcap.get(3)  # float for width of frame in mm
height = vcap.get(4)  # float for height of frame in mm

calibration_matrix_path = args["K_Matrix"]
distortion_coefficients_path = args["D_Coeff"]

k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)

# fps
pf = 0  # time to process previous frame
nf = 0  # time to process current frame

# elapsed time
sf = time.time()


while True:
    ret, frame = video.read()

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

    # write fps into xls file with timestamp
    xf = nf - sf

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
    rx1.append(newR1)
    rx2.append(newR2)
    rx3.append(newR3)
    ts.append(xf)
    lineR1.set_data(ts,rx1)
    lineR2.set_data(ts,rx2)
    lineR3.set_data(ts,rx3)
    ax.set_ylim((-5,5))
    ax.set_xlim((xf-105,(xf+5)))

    fig.canvas.draw()
    
    plt.pause(.01)
    cv2.imshow('Estimated Pose', output)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
video.release()

plt.savefig("RadPlot-"+dateStr+".png")