import numpy as np
import imutils
import cv2
import sys
import time
from math import pow, sqrt
#this class is used to sort the data in an easy to read way
class importantMath :
    #this is the function that runs when you initiate the importantMath class
    def __init__(self, image):
        self.__objLoc = (0,0)
        self.minRadSize = 20
        self.root = 0
        self.objRad = 0
        self.objTooCloseRad = 90
        self.objTooFarRad = 40
        self.objTooFarRad_notInSafeZone = 100
        self.objTooCloseRad_notInSafeZone = 60
        #image size
        height,width,color = image.shape
        #center of image
        heightDiv2 = height / 2
        widthDiv2 = width / 2 
        self.centerPage = (int(widthDiv2), int(heightDiv2))
        #initiating color <default> red range ;; will make options for blue incase we encounter some problems
        self.__color = {
            'red'    : {'low' : {'R':17,'G': 15,'B':100}, 'high': {'R': 50,'G': 56,'B':200}},
            'blue'   : {'low' : {'R':86,'G': 31,'B':  4}, 'high': {'R':220,'G': 88,'B': 50}}}
        #default is set to purple
        self.defaultColor = self.__color['blue']
        #creates a data point that says if there is an  object tracked
        self.tracked = False
        self.safeAreaRadius = 100
        self.prevObjRad = 0
        #100 = 100%
        self.ratio = {'objRadPercentRatio' :0.15, 'midPercentRatio' : 0.20, 'safeRadPercentRatio': 0.65 }
        self.minSafeRadSize = 95
    #calculate safe area zone
    def safeAreaZone(self):
        #get the size of the obj radius
        if self.tracked:
           #full Dist
           dist = self.__internal_distFromCenter()
           self.safeAreaRadius = dist - (dist * self.ratio["objRadPercentRatio"] + dist * self.ratio['midPercentRatio'])
           if self.safeAreaRadius <= self.minSafeRadSize:
                self.safeAreaRadius = self.minSafeRadSize

    #-- return safe zone radius
    def get_safe_zone_rad(self):
        return int(self.safeAreaRadius)

    #sets the color that will be recognized    
    def updateColor(self, color):
        if color is 'red':
            self.defaultColor = self.__color['purple']
        elif color is 'blue':
            self.defaultColor = self.__color['blue']
        else:
            return False

    #this function updates the object location
    def updateObjLoc(self, obj, rad):
        self.__objLoc = obj
        self.prevObjRad = self.objRad
        self.objRad = rad
        self.__distance_from_center()

    #updates min rad size of a detectable object
    def updateMinRadSize(self,size):
        self.minRadSize = size

    #this function returns the current object location after centering the (0,0) point
    def get_objLoc(self):
        return (self.__objLoc[0] - self.centerPage[0], self.centerPage[1] - self.__objLoc[1])

    #this function returns the raw values of the object location
    def get__objLoc(self):
        return (self.__objLoc[0], self.__objLoc[1])

    #this function calculates the distance of the objects from the center (2d)
    def __distance_from_center(self):
        x1,y1 = self.__objLoc
        x2,y2 = (self.centerPage[0],self.centerPage[1])
        power = pow(x1-x2,2) + pow(y1-y2,2)
        self.root = sqrt(power)

    #this function returns the distance that will be used to sinmulate the 3d distance (the 2d distance - the object radius and the center radius)
    def dist_minus_rads(self):
        self.sim3Ddistance = self.root - (self.objRad + self.safeAreaRadius)
        return self.sim3Ddistance

    #this functions returns the distance of the object from the center
    def return_distance_from_center(self):
        return self.root
    def __internal_distFromCenter(self):
        self.__distance_from_center()
        return self.root
    #this function is used to check the data that is calculated or stored
    def printData(self):
        print(self.objRad, self.prevObjRad, self.safeAreaRadius)
        #print(self.tracked)
        #print("object location no sim", self.get__objLoc(), "\n","objective location sim", self.get_objLoc())
        #print("distance from mid", self.return_distance_from_center(), "\n", "distance without rad\n aka sim 3d dist", self.dist_minus_rads(), "\n\n")

