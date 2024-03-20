import numpy as np
import cv2

# Load the left and right images
imgL = cv2.imread('./data/pexels-photo-9977959.webp')
imgR = cv2.imread('./data/view-over-the-river-at-dusk.webp')

# Convert images to grayscale
imgGrayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
imgGrayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

# Stereo matching parameters
minDisp = 32
numDisp = 144 - minDisp
windowSize = 5

# Create StereoSGBM object
stereo = cv2.StereoSGBM_create(
    minDisparity=minDisp,
    numDisparities=numDisp,
    blockSize=16,
    P1=8*3*windowSize**2,
    P2=32*3*windowSize**2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32
)

# Calculate disparity
disparity = stereo.compute(imgGrayL, imgGrayR).astype(np.float32) / 16
# disparity = stereo.compute(imgL, imgR).astype(np.float32) / 16
disparity = (disparity - minDisp) / numDisp

# Display images and disparity map
cv2.imshow("Left Image", imgGrayL)
cv2.imshow("Right Image", imgGrayR)
cv2.imshow("Disparity Map", disparity)

cv2.waitKey(0)
cv2.destroyAllWindows()


