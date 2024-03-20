import cv2
import numpy as np

class DepthEstimate:
    # Define stereo matching parameters and other class variables
    minDisparity = 16
    numDisparities = 192 - minDisparity
    blockSize = 5
    uniquenessRatio = 1
    speckleWindowSize = 3
    speckleRange = 3
    disp12MaxDiff = 200
    P1 = 600
    P2 = 2400
    prefilterCap = 0

    def __init__(self):
        # Initialize any necessary variables or objects
        self.imgL = None
        self.imgR = None

    def capture_img(self):
        # Capture images from the camera and save them to files
        cam = cv2.VideoCapture(0)

        cv2.namedWindow("test")
        img_count = 0

        while True:
            ret, frame = cam.read()
            if not ret:
                print("Failed to grab frame")
                break
            cv2.imshow("test", frame)

            k = cv2.waitKey(1)
            if k == 27:  # Press ESC to exit
                print("Escape pressed, closing...")
                break
            elif k == 32:  # Press SPACE to capture and save image
                img_name = "opencv_frame_{}.jpg".format(img_count)
                cv2.imwrite(img_name, frame)
                print("{} is saved to the directory".format(img_name))
                img_count += 1

        cam.release()
        cv2.destroyAllWindows()

    def load_img(self):
        # Load images from files
        self.imgL = cv2.imread('opencv_frame_0.jpg', cv2.IMREAD_GRAYSCALE)
        self.imgR = cv2.imread('opencv_frame_1.jpg', cv2.IMREAD_GRAYSCALE)
        print("Image loaded")

    def stereomatching(self):
        # Create StereoSGBM object with the defined parameters
        stereo = cv2.StereoSGBM_create(
            minDisparity=self.minDisparity,
            numDisparities=self.numDisparities,
            blockSize=self.blockSize,
            uniquenessRatio=self.uniquenessRatio,
            speckleRange=self.speckleRange,
            speckleWindowSize=self.speckleWindowSize,
            disp12MaxDiff=self.disp12MaxDiff,
            P1=self.P1,
            P2=self.P2,
            preFilterCap=self.prefilterCap)

        # Perform stereo matching on loaded images
        disparity = stereo.compute(self.imgL, self.imgR)

        # Print depth data
        baseline = 100  # Example baseline value (in mm)
        focal_length = 500  # Example focal length value (in pixels)
        depth = (baseline * focal_length) / disparity

        print("Depth data (in mm):")
        print(depth)

        # Display disparity map
        cv2.imshow('Disparity', disparity)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

# Create an instance of the DepthEstimate class
depth_estimator = DepthEstimate()

# Capture images or load existing images
depth_estimator.capture_img()
depth_estimator.load_img()

# Perform stereo matching
depth_estimator.stereomatching()
