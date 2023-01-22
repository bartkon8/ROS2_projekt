#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library
import numpy as np
from geometry_msgs.msg import Twist
import sys
import argparse


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


class ProjectNode(Node):
	def __init__(self):
		super().__init__('Project_Node')
		self.br = CvBridge()
		self.control_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.camera_publisher_ = self.create_publisher(Image, 'aruco_frames', 10)
		self.vel_subscriber_ = self.create_subscription(Twist, 'vel_sub', self.sub_callback, 10)
		self.camera_subscriber_ = self.create_subscription(Image, 'video_frames',self.camera_callback, 10)
		self.get_logger().info("Control Node has been started")
		self.arucoParams = cv2.aruco.DetectorParameters_create()
		self.aruco_type = "DICT_4X4_100"
		self.arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_type])
		self.cY = 0.0

	def aruco_display(self, corners, ids, rejected, image):

		speeds = Twist()
		if len(corners) > 0:

			ids = ids.flatten()

			for (markerCorner, markerID) in zip(corners, ids):
				print(markerCorner)
				corners = markerCorner.reshape((4, 2))
				print(corners)
				(topLeft, topRight, bottomRight, bottomLeft) = corners

				topRight = (int(topRight[0]), int(topRight[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))

				cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
				cv2.line(image, topLeft, bottomLeft, (0, 255, 0), 2)
				cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
				cv2.line(image, bottomLeft, bottomRight, (0, 255, 0), 2)

				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				self.cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				cv2.circle(image, (cX, self.cY), 4, (0, 0, 255), -1)

				cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
							0.5, (0, 255, 0), 2)
				print(f"[Inteference] ArUco marker IF: {markerID}")

				if self.cY > 325:
					speeds.linear.x = -30.0
					self.control_publisher_.publish(speeds)
				else:
					speeds.linear.x = 30.0
					self.control_publisher_.publish(speeds)
		else:
			speeds.linear.x = 0.0
			self.control_publisher_.publish(speeds)
		return image

	def camera_callback(self, video):
		current_frame = self.br.imgmsg_to_cv2(video)
		h, w, _ = current_frame.shape
		width = 1000
		height = int(width * (h / w))
		img = cv2.resize(current_frame, (width, height), interpolation=cv2.INTER_CUBIC)
		corners, ids, rejected = cv2.aruco.detectMarkers(img, self.arucoDict, parameters=self.arucoParams)
		detected_markers = self.aruco_display(corners, ids, rejected, img)
		speeds = Twist()
		detected_markers = self.br.cv2_to_imgmsg(detected_markers)
		self.camera_publisher_.publish(detected_markers)


	def sub_callback(self, message):
		self.get_logger().info(f"X: {message.linear.x}\nY: {message.linear.y}\nZ: {message.linear.z}")

def main():
	rclpy.init()
	node = ProjectNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

