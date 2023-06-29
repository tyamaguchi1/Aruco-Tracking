# Aruco Tracking
 Repo for all files related to aruco markers/tracking for behavior experiment

Files
1. checker5 - take pictures of a predictable pattern such as a checkerboard at various distances and angles using the camera you will use for the Aruco tracking in order to calibrate your camera.
2. calibration.py - run this calibration algorithm to calibrate your camera using the folder of pictures.
3. library.py - library of Aruco marker codes that the "tracking" scripts accesses to recognize the marker ID
4. Tracking_Center.py - main script: recognizes Aruco markers, identifies the 3 markers closest to the center of the frame, tracks the rotation/translation of those 3 IDs, and plots the rotation/translation of those IDs
5. Tracking_Tilt.py - main script: recognizes Aruco markers, identifies the 3 markers with the smallest tilt, tracks the rotation/translation of those 3 IDs, and plots the rotation/translation of those IDs
6. calibration_matrix.npy - camera calibration output used in calculating translation/rotation
7. distortion_coefficients.npy - distortion calibration used to reduce effects of edge distortion in video
