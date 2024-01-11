import math
#[id, value to pass to VideoCapture, rotation (radians), x (mm), y (mm), calibration matrix, distortion coefficients]

CamArray = [
    [0, 0, math.pi, 405, 250, "cam_1_calibration_matrix.npy", "cam_1_distortion_coefficients.npy"],
    [1, 1, 3*math.pi/2, 260, 95, "cam_3_calibration_matrix.npy", "cam_3_distortion_coefficients.npy"],
    [2, 2, -(52.5*math.pi)/180, 105, 450, "cam_2_calibration_matrix.npy", "cam_2_distortion_coefficients.npy"]
]