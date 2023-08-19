#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
from cv2 import aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}

    Detected_ArUco_markers = {}
    ## enter your code here ##

    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()

    corners, ids, _ = aruco.detectMarkers(gray_img, aruco_dict, parameters = parameters)

    for i in range(len(ids)) :
        set_of_corners = corners[i][0]
        Detected_ArUco_markers.update({ids[i][0] : set_of_corners})

    return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

    ArUco_marker_angles = {}
	## enter your code here ##

    for tag in Detected_ArUco_markers.items() :
        _id, corners = tag
        tl = corners[0]
        tr = corners[1]

        vector = (tr - tl)

        if vector[0] == 0 : 
            angle = [90, 270][vector[1] < 0]
        elif vector[1] == 0 : 
            angle = [0, 180][vector[0] < 0]
        else :
            angle = math.degrees(math.atan(vector[1] / vector[0]))
            angle = [180 + angle, angle][vector[0] > 0]
        
        ArUco_marker_angles.update({_id : (-angle + 90) % 360})

    return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement

    ## enter your code here ##
    img = img.copy()

    distance = 100

    for item in Detected_ArUco_markers.items() :
        _id, corners = item
        angle = ArUco_marker_angles[_id]

        center = 0.5 * (corners[0] + corners[2])

        cv2.circle(img, tuple(corners[0].astype(int)), 5, (125, 125, 125), -1)
        cv2.circle(img, tuple(corners[1].astype(int)), 5, (  0, 255,   0), -1)
        cv2.circle(img, tuple(corners[2].astype(int)), 5, (180, 105, 255), -1)
        cv2.circle(img, tuple(corners[3].astype(int)), 5, (255, 255, 255), -1)

        cv2.circle(img, tuple(center.astype(int)),     5, (  0,   0, 255), -1)

        cv2.line(img, tuple(center.astype(int)), tuple((0.5 * (corners[0] + corners[1])).astype(int)), (255,   0,   0), 6)

        cv2.putText(img, str(int(angle)), tuple((center - np.array([distance, 0])).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (  0, 255,   0), 2)
        cv2.putText(img, str(_id),   tuple((center + np.array([distance / 4, 0])).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (  0,   0, 255), 2)

    return img


