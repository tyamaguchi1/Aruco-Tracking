# Aruco Tracking
 Repo for all files related to aruco markers/tracking for behavior experiment

Files
1. checker5 - take pictures of a predictable pattern such as a checkerboard at various distances and angles using the camera you will use for the Aruco tracking in order to calibrate your camera.
2. calibration.py - run this calibration algorithm to calibrate your camera using the folder of pictures.
3. library.py - library of Aruco marker codes that WORKS.py accesses to recognize the marker ID
4. Aruco_Tracking.py - main script to recognize Aruco markers, identify the marker closest to the center of the frame,  track its rotation/translation vectors, and plot the rotation/translation vectors of interest
5. graphAngles.py - preliminary script to track the rotation vector of the marker and produce a 3x3 rotation matrix
6. graphRadians.py - preliminary script to track the rotation vector of the marker and produce a 1x3 rotation vector in radians
