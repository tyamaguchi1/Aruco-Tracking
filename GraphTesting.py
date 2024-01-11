# Import libraries
import matplotlib.pyplot as plt
import numpy as np
import cv2
import math
from config import CamArray 

fig = plt.figure(figsize = (14, 8))

CAM_FOV =37 #In degrees. Either 48 or 37
MARKER_NUM = 10
ARENA_RADIUS = 30 #mm

#Returns [lower bound, upper bound]
def GetIQRRange(arr):
    medpos = ((len(arr)+1)/2)-1
    lowpos = ((math.floor(medpos-0.5)+1)/2)-1
    lower = 0
    if lowpos % 2 == 0:
        lower = arr[lowpos]
    else:
        lower = (arr[math.floor(lowpos)]+arr[math.ceil(lowpos)])/2
    highpos = ((math.floor(medpos-0.5)+1)/2)+math.floor(medpos)-1
    high = 0
    if highpos % 2 == 0:
        high = arr[highpos]
    else:
        high = (arr[math.floor(highpos)]+arr[math.ceil(highpos)])/2
    IQR = high-lower
    return [lower-1.5*IQR, high+1.5*IQR]


#set limits as 10 to each side of the farthest away points of: camera locations, bounding points where camera views overlap
points = []
boundingEquations = []
#Create the bounding equations for each camera
for camera in CamArray:
    points.append([camera[3], camera[4]])
    boundingEquations.append([camera[4], math.tan(camera[2]+(CAM_FOV*math.pi / 360)), camera[3]])
    boundingEquations.append([camera[4], math.tan(camera[2]-(CAM_FOV*math.pi / 360)), camera[3]])
#Get array of all intersections in camera views
intersections = []
for bound1 in boundingEquations:
    for bound2 in boundingEquations:
        if bound1 != bound2:
            temp = ((bound1[1]*bound1[2]) - (bound2[1]*bound2[2]) + bound2[0] - bound1[0]) / (bound1[1] - bound2[1])
            intersections.append([temp, bound1[1]*(temp-bound1[2]) + bound1[0]])
#Get array of intersections without outliers
cleanInts = []
xInts = []
yInts = []
for inter in intersections:
    xInts.append(inter[0])
    yInts.append(inter[1])
xInts.sort()
yInts.sort()
xRange = GetIQRRange(xInts)
yRange = GetIQRRange(yInts)
print("x range:",xRange)
print("y range:",yRange)
for inter in intersections:
    if inter[0] > xRange[0] and inter[0] < xRange[1] and inter[1] > yRange[0] and inter[1] < yRange[1]:
        cleanInts.append(inter)
    else:
        print("cleaned out",inter)
#Get range
xInts = []
yInts = []
for inter in cleanInts:
    xInts.append(inter[0])
    yInts.append(inter[1])
xInts.sort()
yInts.sort()

xmin = xInts[0]-10
xmax = xInts[len(xInts)-1]+10
ymin = yInts[0]-10
ymax = yInts[len(yInts)-1]+10

# Add features to our figure
plt.legend()
plt.grid(True, linestyle =':')
x = np.linspace(xmin, xmax, 50)
permPlots = []
permPoints = []
radspace = np.linspace(0,2*math.pi, 16)
 
plt.title('Arena Position')
plt.xlabel('x (mm)')
plt.ylabel('y (mm)')

for eq in boundingEquations:
    permPlots.append([x, eq[1]*(x-eq[2])+eq[0], "b:"])
for eq in intersections:
    permPoints.append([eq[0], eq[1], 5, "blue"])

while True:
    plt.clf()
    
    #updating graph logic
   
    #chores
    for plot in permPlots:
        plt.plot(plot[0], plot[1], plot[2])
    for plot in permPoints:
        plt.plot(plot[0], plot[1], marker="o", markersize=plot[2], markerfacecolor=plot[3])
    plt.axis([xmin, xmax, ymin, ymax])
    plt.pause(.001)

    if 0xFF == ord('q'):    
        print("exiting")
        break