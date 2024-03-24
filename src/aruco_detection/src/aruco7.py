#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def aruco_display(corners, ids, rejected, image):
    # Function to display ArUco markers
    # Implementation omitted for brevity
    pass

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    detected_markers = aruco_display(corners, ids, rejected_img_points, frame)

    if len(corners) > 0:
        for i in range(0, len(ids)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                           distortion_coefficients)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
            # Publish the ArUco marker ID and pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera_frame"
            pose_msg.pose.position.x = tvec[0][0][0]
            pose_msg.pose.position.y = tvec[0][0][1]
            pose_msg.pose.position.z = tvec[0][0][2]
            pose_msg.pose.orientation.x = rvec[0][0][0]
            pose_msg.pose.orientation.y = rvec[0][0][1]
            pose_msg.pose.orientation.z = rvec[0][0][2]
            if len(rvec[0][0]) > 3:
                pose_msg.pose.orientation.w = rvec[0][0][3]
            pose_publisher.publish(pose_msg)

def main():
    rospy.init_node("aruco_detection")
   
    cap = cv2.VideoCapture(0)
    global pose_publisher
    pose_publisher = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        intrinsic_camera = np.array([[933.15867, 0, 657.59],
                                      [0, 933.1586, 400.36993],
                                      [0, 0, 1]])
        distortion = np.array([-0.43948, 0.18514, 0, 0])
        for aruco_type in ARUCO_DICT.values():
            pose_estimation(frame, aruco_type, intrinsic_camera, distortion)
        cv2.imshow("Estimated Pose", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
