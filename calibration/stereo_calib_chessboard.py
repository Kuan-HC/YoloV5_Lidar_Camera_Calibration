import cv2
import numpy as np
import cv2 as cv
import glob
import os
from natsort import natsorted

################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (8,6)
frameSize = (640,512)

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 20
objp = objp * size_of_chessboard_squares_mm

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.

# imagesLeft=glob.glob('images/stereoleft/V4_testL/*.png')
# imagesRight=glob.glob('images/stereoright/V4_testR/*.png')
imagesLeft=natsorted(glob.glob('T_RGB_images/left/test/*.png'))
imagesRight=natsorted(glob.glob('T_RGB_images/right/test/*.png'))

num=0
i=0
for imgLeft, imgRight in zip(imagesLeft, imagesRight):

    imgL = cv.imread(imgLeft)
    # imgL = imgL[620:1520, 900:1840]  # [up, down, left, right]
    # imgL = cv2.resize(imgL, [640, 512], interpolation=cv2.INTER_AREA)
    imgR = cv.imread(imgRight)
    c, w, h = imgR.shape[::-1]
    frameSize = (w, h)
    ''' write image'''
    # cv2.imwrite(
    #     '/home/nawaz/HKPC_final_files/Thermal_Cognitive_V0.5/Thermal_Cognitive/src/image_fusion/src/T_RGB_images/left/V4/image_00' + str(
    #         i) + '.png', imgL)
    # i += 1
    ''' gray images'''
    grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    '''Find the chess board corners'''
    ''' For RGB camera'''
    # retL, cornersL = cv.findChessboardCorners(grayL, chessboardSize, None)
    # retR, cornersR = cv.findChessboardCorners(grayR, chessboardSize, None)
    ''' for thermal cameras'''
    checkerboard_flags = 0
    retL, cornersL = cv.findChessboardCorners(grayL, chessboardSize, flags = cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | checkerboard_flags)
    retR, cornersR = cv.findChessboardCorners(grayR, chessboardSize, flags = cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | checkerboard_flags)

    if retL and retR == True:

        objpoints.append(objp)
        cornersL = cv.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)
        cornersR = cv.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(cornersR)

        # Draw and display the corners
        cv.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
        cv.imshow('img left', imgL)
        cv.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)
        cv.imshow('img right', imgR)
        # frame = np.concatenate((imgL, imgR), axis=1)  # adding frames
        # cv.imshow('img left and right', frame)
        # cv.imwrite(
        #     '/home/nawaz/HKPC_final_files/Thermal_Cognitive_V0.5/Thermal_Cognitive/src/image_fusion/src/T_RGB_images/results/R1_00' + str(
        #         num) + '.png', frame)
        # num +=1
        cv.waitKey(3000)
cv.destroyAllWindows()


'''CALIBRATION '''
''' Left Camera'''
retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(objpoints, imgpointsL, frameSize, None, None)
heightL, widthL, channelsL = imgL.shape
newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))
''' Right camera '''
retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(objpoints, imgpointsR, frameSize, None, None)
heightR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))


''' Stereo Vision Calibration '''
flags = 0
# flags |= cv.CALIB_FIX_INTRINSIC
flags |= cv2.CALIB_FIX_INTRINSIC
# flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
flags |= cv2.CALIB_USE_INTRINSIC_GUESS
flags |= cv2.CALIB_FIX_FOCAL_LENGTH
# flags |= cv2.CALIB_FIX_ASPECT_RATIO
flags |= cv2.CALIB_ZERO_TANGENT_DIST

# Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
# Hence intrinsic parameters are the same
criteria_stereo= (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)

''' Stereo Rectification '''
rectifyScale= 1
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], rot, trans, rectifyScale,(0,0))

stereoMapL = cv.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv.CV_16SC2)
stereoMapR = cv.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv.CV_16SC2)

print("Saving parameters!")
cv_file = cv.FileStorage('stereoMap.xml', cv.FILE_STORAGE_WRITE)

cv_file.write('stereoMapL_x',stereoMapL[0])
cv_file.write('stereoMapL_y',stereoMapL[1])
cv_file.write('stereoMapR_x',stereoMapR[0])
cv_file.write('stereoMapR_y',stereoMapR[1])

cv_file.release()

