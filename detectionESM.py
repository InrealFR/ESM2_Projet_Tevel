"""
Created on 2022
@author: Hugo PONS & Jean-Nicolas WEBER
"""

import os
import sys
import time
from datetime import datetime
import cv2
import cv2.aruco as aruco
import numpy as np
from pymavlink import mavutil

class Detection:
    def __init__(self, camera):
        # Booléens
        self.aruco_found = False
        self.square_found = False
        # Camera
        self.camera = camera
        self.camera_res = (1920, 1080)  # TODO : trouver la bonne res
        # self.camera.framerate = 32
        # self.rawCapture = PiRGBArray(self.camera, size=(640, 480)) (package Picamera nécessaire)
        # Aruco
        self.marker_size = 25  # en cm, on prend un gros marqueur pour être tranquille (quasi aucune detection de carrés blancs)

        # Resolution
        # Focal length and sensors dimensions for Pi camera
        # See: https://www.raspberrypi.com/documentation/accessories/camera.html
        focal_length = 3.60  # Focal length [mm]
        horizotal_res = 1920  # Horizontal resolution (x dimension) [px]
        vertical_res = 1080  # Vertical resolution (y dimension) [px]
        sensor_length = 3.76  # Sensor length (x dimension) [mm]
        sensor_height = 2.74  # Sensor length (y dimension) [mm]
        self.dist_coeff_x = sensor_length / (focal_length * horizotal_res)
        self.dist_coeff_y = sensor_height / (focal_length * vertical_res)
        self.x_imageCenter = int(horizotal_res / 2)
        self.y_imageCenter = int(vertical_res / 2)

        # Ecriture dossiers
        self.dossier = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
        self.path = os.path.join("/home/admin-drone/Bureau/vtol/VTOL_vision/data", self.dossier)
        os.mkdir(self.path)  # création d'un dossier pour stocker les nouvelles images
        # dictionnaire aruco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.parameters = aruco.DetectorParameters_create()
        self.closeToAruco = False
        self.compteur = 0

    def Detection_aterr(self, latitude, longitude, altitude, direction, id_to_test):

        # INITIALISATION
        nbLarge = 5
        nbHaut = 3
        self.compteur += 1
        start_time = time.time()
        self.aruco_found = self.square_found = True
        x_pixel = y_pixel = None
        arucoId = id_to_test
        capture, imageAnalysee = self.camera.read()  # recuperation de la vidéo générée auparavant
        blur = cv2.GaussianBlur(imageAnalysee, (5, 5), 0)  # Gaussian blur filter : removing noise

        # passage en HLS
        hlsFrame = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)

        # création du masque de détection du noir
        lower_bound = (0, 0, 0)  # white color in HLS space
        upper_bound = (255, 70, 255)
        mask_hls = cv2.inRange(hlsFrame, lower_bound, upper_bound)
        """cv2.imshow('blur', blur)"""
        """cv2.imshow('hls', mask_hls)"""

        # trouver les éléments noirs dans l'image
        contours, hierarchy = cv2.findContours(mask_hls,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        # analyse des éléments noirs et trouver le plus grand
        i = 0
        maxi = 0
        [x, y, w, h] = [0, 0, 0, 0]
        [x1, y1, w1, h1] = [0, 0, 0, 0]
        [x2, y2, w2, h2] = [0, 0, 0, 0]
        imageAnalysee2 = imageAnalysee.copy()
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > maxi):
                maxi = area
                i = i + 1
                x1, y1, w1, h1 = cv2.boundingRect(contour)

            w = w1
            h = h1
        cropped = imageAnalysee2[y1:y1 + h1, x1:x1 + w1]  # isolation de la zone noire la plus grande sur une image
        imageAnalysee2 = cv2.rectangle(imageAnalysee2, (x1, y1),
                                       (x1 + w1, y1 + h1),
                                       (255, 255, 255), 6)

        caseHor = w / nbLarge
        caseVer = h / nbHaut

        # passage en HSV
        hsvFrame = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # création du masque de détection du vert
        green_lower = np.array([25, 52, 72], np.uint8)
        green_upper = np.array([102, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

        kernal = np.ones((5, 5), "uint8")

        green_mask = cv2.dilate(green_mask, kernal)

        # trouver les éléments verts dans l'image
        contours, hierarchy = cv2.findContours(green_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        # analyse des éléments verts et indication de ceux -ci sur l'image
        i = 0
        ligne = 0
        colonne = 0

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 1000):
                i = i + 1
                x2, y2, w2, h2 = cv2.boundingRect(contour)
                imageAnalysee2 = cv2.rectangle(imageAnalysee2, (x1 + x2, y1 + y2),
                                               (x1 + x2 + w2, y1 + y2 + h2),
                                               (0, 255, 0), 6)
                if (i % nbLarge == 0):
                    ligne = ligne + 1
                    colonne = 0
                else:
                    colonne = colonne + 1

        X = int(colonne * caseHor + (caseHor - w2) / 2)
        Y = int(ligne * caseVer + (caseVer - h2) / 2)
        X1 = int((colonne + 1) * caseHor - (caseHor - w2) / 2)
        Y1 = int((ligne + 1) * caseVer - (caseVer - h2) / 2)
        midY = int((Y + Y1) / 2)
        midX = int((X + X1) / 2)

        x_pixel = Xas = x1 + midX
        y_pixel = Yas = y1 + midY

        imageAnalysee2 = cv2.rectangle(imageAnalysee2, (x1 + X, y1 + Y),
                                       (x1 + X1, y1 + Y1),
                                       (0, 0, 255), 2)

        cv2.putText(imageAnalysee2, 'emplacement suivant',
                    (x1 + X, y1 + midY),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    2,
                    (0, 255, 255))

        cv2.imshow('final', imageAnalysee2)
        return (x_pixel, y_pixel, self.aruco_found, self.square_found)  # on renvoie les deux booleens et nos pixels cible
        # Si nos deux booleens sont faux : on n'a ni détécté d'aruco et de carré blanc, on continue la mission
        # Si aruco_found est vrai : on s'asservit dessus, c'est gagné
        # Si square_found est vrai : on s'asservit dessus et on vérifie que c'est bien un Aruco
# vid = cv2.VideoCapture(0)
# boucle = Detection(vid)
# while (True):
#     time.sleep(0.1)
#     capture, frame = vid.read()
#     cv2.imshow('camera', frame)
#     x_centerPixel_target, y_centerPixel_target, aruco_found, square_found = boucle.Detection_aterr(0,0,0,0,0)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break