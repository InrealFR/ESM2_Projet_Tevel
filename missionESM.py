#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022

@author: Hugo PONS & Jean-Nicolas WEBER
"""
import os
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time
import math

from threading import Thread
import threading

from math import atan2, cos, radians, sin, sqrt, pi
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array
from datetime import datetime
from detectionESM import Detection
from commande_droneESM import Drone
import sys
import time

#--------------------- Parametres du vehicule ----------------------------
vitesse = .3 #m/s
altitudeDeVol = 15
research_whiteSquare = True
distanceAccuracy = 2 # rayon en metre pour valider un goto

# Boolean variables
global found

# Counter variables
global counter_no_detect
counter_no_detect = 0

global x_centerPixel_target
x_centerPixel_target =  None
global y_centerPixel_target
y_centerPixel_target =  None
global x_imageCenter
x_imageCenter = 0
global y_imageCenter
y_imageCenter = 0
global altitudeAuSol
global altitudeRelative
global longitude
global latitude
def asservissement(drone_object, detection_object, last_errx, last_erry, errsumx, errsumy,altitudeAuSol,x_centerPixel_target, y_centerPixel_target):

  # PID Coefficients
  kpx = 0.005
  kpy = 0.005
  kdx = 0.0001  # 0.00001 working "fine" for both
  kdy = 0.0001
  kix = 0.000001  # 0.0000001
  kiy = 0.000001
  
  vx = vy = vz = 0

  if altitudeAuSol < 5 : #on asservit moins violent quand on se raproche du sol
    kpx = 0.003
    kpy = 0.003
  else:
    kpx = 0.005
    kpy = 0.005

  print("Pixels values - x:%s - y:%s" % (x_centerPixel_target, y_centerPixel_target))
  
  if x_centerPixel_target == None or y_centerPixel_target == None : # echec Detection
    if counter_no_detect > 10 :   #on fixe le nombre d'image consecutive sans Detection pour considerer qu il ne detecte pas
      if altitudeAuSol > 30 :  # arrêt du drone si il est trop haut
        drone_object.set_velocity(0, 0, 0, 1)
        print ("[asserv] altitudeRelative > 30")
        return 0, 0, 0, 0
    
      else :  # pas de Detection Drone prend de l'altitude => champ de vision plus large => détection plus aisée
        vx = vy =  0
        vz = -1
        drone_object.set_velocity(vx, vy, vz, 1)
        #
    elif counter_no_detect > 1 :   # fixer la position du Drone en cas de non detection pour s'assurer qu'il traite l'image un certain nombre de fois
      vx = vy = vz = 0
      drone_object.set_velocity(vx, vy, vz, 1)   
    errx = 0
    erry = 0
    errsumx = 0
    errsumy = 0
    
  else :  # Detection ok
    print ("[asserv] detection OK")
    errx = 324 - x_centerPixel_target #le déplacement à effectuer en x (pos de notre caméra - pos du carré sur la camera)
    erry = 224 - y_centerPixel_target
    
    dist_center = math.sqrt(errx**2+erry**2)
    dist_angle = atan2(erry, errx)
    heading = drone_object.vehicle.attitude.yaw
    alpha = dist_angle + heading
    errx = dist_center * cos(alpha)
    erry = dist_center * sin(alpha)
    if abs(errx) <= 30:   #marge de 30x10pxl pour considerer que la cible est au centre de l image
      errx = 0
    if abs(erry) <= 10:
      erry = 0
        
    # PD control
    dErrx = (errx - last_errx)# / delta_time
    dErry = (erry - last_erry)# / delta_time
    errsumx += errx# * delta_time
    errsumy += erry# * delta_time
    
    vx = (kpx * errx) + (kdx * dErrx) + (kix * errsumx)
    vy = (kpy * erry) + (kdy * dErry) + (kiy * errsumy)
    vz = 0
    
    if altitudeAuSol < 3 :
      vz = 0.1  # a changer pour descendre
    elif altitudeAuSol > 9 :
      vz = 1  # a changer pour descendre
    elif altitudeAuSol > 5:
      vz = 0.5
    else:
      vz = 0.25

    # Establish limit to outputs
    vx = min(max(vx, -5.0), 5.0)
    vy = min(max(vy, -5.0), 5.0)
    vx = -vx # High opencv is south Dronekit
    
    # Dronekit
    # X positive Forward / North
    # Y positive Right / East
    # Z positive down
    
    if altitudeAuSol < 2 :
      dist_center_threshold = 50
    
    else :
      dist_center_threshold = 300
    if altitudeAuSol <= 1:
      print("Vehicle in LAND mode")
      drone_object.vehicle.mode = VehicleMode("LAND")
      drone_object.set_velocity(0,0,0.1,1)
      while (drone_object.vehicle.location.global_relative_frame.alt !=0 & drone_object.vehicle.armed != True):
       pass
      print("Aterrissage réussi. Arrêt moteurs.")
      drone_object.vehicle.armed = False
      drone_object.vehicle.close()
    if dist_center <= dist_center_threshold :
      drone_object.set_velocity(vy, vx, vz, 1) 
      #print("vy : "+str(vy)+" vx : "+str(vx)+" vz : "+str(vz)+" dist_center <= 30"	
    else :
      #lancer un deplacement pour se rapprocher du centre sans descendre ou monter
      vz = 0
      drone_object.set_velocity(vy, vx, vz, 1)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'
      #print("vy : "+str(vy)+" vx : "+str(vx)+" vz : "+str(vz)+" dist_center decale")

  # Return last errors and sums for derivative and integrator terms
  print(" errx :" + str(errx) + " erry : " + str(erry)+"\n")
  
  return errx, erry, errsumx, errsumy








#---------------------------------------------MISSION PRINCIPALE--------------------------------------

def mission_largage(drone_name, truck):
  last_errx = 0
  last_erry = 0
  errsumx = 0
  errsumy = 0
  vid = cv2.VideoCapture(0)
  counter_no_detect = 0
  altitudeAuSol = 0.0
  print("[mission] Mission started!")
  drone_object = Drone()    #permet de connecter le drone via dronekit en creant l objet drone
  detection_object = Detection(vid)  # creer l objet detection
  #########verrouillage servomoteur et procedure arm and takeoff
  print("[mission] Launching and take off routine...")
 
  drone_object.lancement_decollage(20, drone_name)

  #########passage en mode AUTO et debut de la mission
  drone_object.passage_mode_Auto()
  
  # a partir d'un certain waypoint declencher le thread de detection
  while drone_object.vehicle.commands.next <= 0:
    pass

  while (drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO"):
    time.sleep(0.1)
    capture, frame = vid.read()
    cv2.imshow('camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
    	break
    # actualisation de l altitude et gps
    print("actualisation gps")
    #altitudeAuSol = drone_object.vehicle.rangefinder.distance #On récupère l'altitude au sol grâce à un capteur (rangefinder) qui regarde vers le bas
    altitudeAuSol = -1*drone_object.vehicle.location.local_frame.down
    print(altitudeAuSol)
    altitudeRelative = drone_object.vehicle.location.global_relative_frame.alt
    longitude = drone_object.vehicle.location.global_relative_frame.lon
    latitude = drone_object.vehicle.location.global_relative_frame.lat
    heading = drone_object.vehicle.attitude.yaw
    
    #le script Detection Target
    x_centerPixel_target, y_centerPixel_target,found = detection_object.Detection_aterr()
    # Asservissement control
    if drone_object.get_mode() == "GUIDED" :
      last_errx, last_erry, errsumx, errsumy = asservissement(drone_object, detection_object, last_errx, last_erry, errsumx, errsumy, altitudeAuSol,x_centerPixel_target, y_centerPixel_target)
    
    if not drone_object.get_mode() == "GUIDED" and not drone_object.get_mode() == "AUTO":
      break
  
    #--------------- Case 1: Cagette  -----------------------
    if found:
      print("[detection] Cagette detectee.")     
      counter_no_detect = 0
      while drone_object.get_mode() != "GUIDED":
        drone_object.set_mode("GUIDED")

      print("x_centerPixel_target : "+str(x_centerPixel_target))
      print("y_centerPixel_target : "+str(y_centerPixel_target))
      print("x_object : "+str(detection_object.x_imageCenter))
      print("y_object : "+str(detection_object.y_imageCenter))
      dist_center = math.sqrt((detection_object.x_imageCenter-x_centerPixel_target)**2+(detection_object.y_imageCenter-y_centerPixel_target)**2)
 
    #--------------- Case 3: No detection ------------
    else:
      print("[detection] Case 3: No detection(%a times)" %counter_no_detect)

      counter_no_detect += 1

      if counter_no_detect > 35:
        print("[mission] 35 times without tag or white detection, not interesting place.")

        while drone_object.get_mode() != "RTL" :
          drone_object.set_mode("RTL")

          # Reset visual PID errors
          last_errx = 0
          last_erry = 0
          errsumx = 0
          errsumy = 0

  if drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO":  #securite pour ne pas que le drone reprenne la main en cas d interruption
    #########repart en mode RTL
    drone_object.set_mode("RTL") #### modif preciser qu on est en guided avant et ajouter l altitude du RTL


mission_largage('futuna',50,False)