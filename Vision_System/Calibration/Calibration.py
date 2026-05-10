import numpy as np
import cv2
import glob

# ----------CONFIGURATION----------
CHESS_SIZE = (9, 6) 
SQUARE_SIZE = 26.0 

IMAGES_FOLDER = "side_images/*.png"
OUTPUT_FILE = "calib_side_data.npz"

#----------POINT PREPARATION----------
objp = np.zeros((CHESS_SIZE[0] * CHESS_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESS_SIZE[0], 0:CHESS_SIZE[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

objpoints = []
imgpoints = []

#----------UPLOAD DIRECTORY----------
images = glob.glob(IMAGES_FOLDER)

print(f"PROCESSING {len(images)}")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #----------FIND CORNERS----------
    ret, corners = cv2.findChessboardCorners(gray, CHESS_SIZE, None)

    if ret:
        objpoints.append(objp)
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        #----------VISUAL CONFIRMATION----------
        cv2.drawChessboardCorners(img, CHESS_SIZE, corners2, ret)
        cv2.imshow('ANALYZING', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

#----------CALIBRATION----------
if len(objpoints) > 0:
    print("CALCULATING PARAMETERS")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if ret:
        #----------SAVE RESULTS----------
        np.savez(OUTPUT_FILE, mtx=mtx, dist=dist)
        
        print("\n--- CALIBRACIÓN EXITOSA ---")
        print(f"Resultados guardados en: {OUTPUT_FILE}")
        print("\nMatriz Intrínseca (mtx):")
        print(mtx)
        print("\nCoeficientes de Distorsión (dist):")
        print(dist)
        
        #----------PROYECTION ERROR----------
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print(f"\nTOTAL ERROR: {mean_error/len(objpoints)}")
    else:
        print("ERROR")
else:
    print("NO CHESSBOARD DETECTED")