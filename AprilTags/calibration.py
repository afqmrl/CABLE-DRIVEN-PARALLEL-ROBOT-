import numpy as np
import cv2 as cv
import glob
import pickle

# Chessboard settings
chessboardSize = (12,7)
frameSize = (640,480)  # Use a higher resolution if possible
size_of_chessboard_squares_mm = 20  # Use accurate size!

# Termination criteria for corner refinement (higher precision)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

# Prepare object points
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2) * size_of_chessboard_squares_mm

objpoints = []  # 3D points
imgpoints = []  # 2D points

# Load images
images = glob.glob('images/*.png')

for image in images:
    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    gray = cv.equalizeHist(gray)  # Improve contrast

    ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

    if ret:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)
        imgpoints.append(corners2)

        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)  # Display for 500ms per image

cv.destroyAllWindows()

# Camera calibration
ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

# Save calibration results
pickle.dump((cameraMatrix, dist), open("calibration.pkl", "wb"))
pickle.dump(cameraMatrix, open("cameraMatrix.pkl", "wb"))
pickle.dump(dist, open("dist.pkl", "wb"))

# Undistortion
img = cv.imread('images/cali5.png')
h, w = img.shape[:2]
newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 0.9, (w,h))

# Undistort with Remapping
mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('caliResult.png', dst)

# Compute Reprojection Error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    print(f"Image {i}: Reprojection Error = {error:.4f}")
    mean_error += error

print(f"Total Average Error: {mean_error / len(objpoints):.4f}")
