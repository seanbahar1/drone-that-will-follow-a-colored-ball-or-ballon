######### TODO #########
# safe area needs to update its size when the object radius changes
# set a timer so if the object is lost for more than 5 sec auto move to rtl mode
# add a class that have all the drone control functions
# add more comments
# try to make the tracking more consistant cause light is a bitch

from collections import deque
from  drone_manual_control import droneCommands
from  imageMath import importantMath
#from imutils.video import VideoStream
import numpy as np
import imutils
import cv2
import sys
import time
from math import pow, sqrt

cap = cv2.VideoCapture(0)
ret, image = cap.read()

tracking = True

#initiating the important math class
data = importantMath(image)
drone = droneCommands("COM4")
# text style For x,y
font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (0, 20)
fontScale = 1
fontColor = (255, 255, 255)
lineType = 2

cv2.namedWindow('RGB')

def sendCommandsMultipleTimesToInsureDelivery(times, delay_sec, func):
    while times > 0:
        times = times - 1
        func()
        time.sleep(delay_sec)


if tracking:

    # initialize the list of tracked points, the frame counter,
    # and the coordinate deltas
    linelength = 16
    pts = deque(maxlen=linelength)
    counter = 0
    (dX, dY) = (0, 0)
    direction = "down"

    count = 0

    while True:
        # Capture frame-by-frame
        (grabbed, image) = cap.read()

        thrs = (data.defaultColor['low'] ['R'],data.defaultColor['low']  ['G'],data.defaultColor['low'] ['B'],
                data.defaultColor['high']['R'],data.defaultColor['high'] ['G'],data.defaultColor['high']['B'])

        colourLower = (thrs[0], thrs[1], thrs[2])
        colourUpper = (thrs[3], thrs[4], thrs[5])

        # resize the frame, blur it, and convert it to the HSV
        # color space
        image = imutils.resize(image, width=600)

        # construct a mask for the object colour, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(image, colourLower, colourUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            data.updateObjLoc((int(x), int(y)), radius)

            # only proceed if the radius meets a minimum size
            if radius > data.minRadSize:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                data.tracked = True
                cv2.circle(image, (int(x), int(y)), int(radius), (100, 100, 100), 2)
                cv2.circle(image, center, 5, (100, 100, 255), -1)
                pts.appendleft(center)
        else:
            data.tracked = False

        # loop over the set of tracked points
        for i in np.arange(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue

        data.printData()
        objLoc_forMovementCalculations = data.get_objLoc 
        data.safeAreaZone()
        safeZoneRad = data.get_safe_zone_rad()

        #-- calculating which commands to send to the drone
        if data.return_distance_from_center() < safeZoneRad:
            #-- this rins when the object is in the safe area
            #-- drone will only move forwards or backwards here (or not at all and just hold itself in the air)
            if data.objRad > data.objTooCloseRad:
                #-- object is too close
                #-- drone will move backwards
                    #-- moving backwards
                    drone.down()

            elif data.objRad < data.objTooFarRad:
                #-- object is too far
                #-- drone will move forward
                    #-- moving forward
                    drone.up()

            else:
                #-- this means the distance is ok and we can stay at the same place
                    #-- this will reset velocity for now
                    drone.reset_velocity() 

        else:
            #-- this runs when the object is outside the safe area

            #-- first checking if forwards or backwards
            if data.objRad > objTooFarRad_notInSafeZone:

                    #-- this means the drone is too far and the movement will be forwards + the sides
                        #-- checking if we need to move to the right or to the left. if either just fowards

                        midPlusSafeZoneRad = data.widthDiv2 + safeZoneRad

                        if objLoc_forMovementCalculations[0] > midPlusSafeZoneRad:
                           #-- will call up and right 3 times [comms are usualy unstable so we send the request multiple times]
                           sendCommandsMultipleTimesToInsureDelivery(drone.requestAmount, drone.requestDelay, drone.up_right())

                        elif objLoc_forMovementCalculations[0] < midPlusSafeZoneRad:
                           #-- will call up and left 3 times [comms are usualy unstable so we send the request multiple times]
                           sendCommandsMultipleTimesToInsureDelivery(drone.requestAmount, drone.requestDelay, drone.up_left())

                        else:
                            sendCommandsMultipleTimesToInsureDelivery(drone.requestAmount, drone.requestDelay, drone.up())
            
            elif data.objRad < objTooFarRad_notInSafeZone:

                    #-- this means the drone is too close and the movement will be backwards + the sides
                        #-- checking if we need to move to the right or to the left. if either just backwards

                        midPlusSafeZoneRad = data.widthDiv2 + safeZoneRad

                        if objLoc_forMovementCalculations[0] > midPlusSafeZoneRad:
                           #-- will call up and right 3 times [comms are usualy unstable so we send the request multiple times]
                           sendCommandsMultipleTimesToInsureDelivery(drone.requestAmount, drone.requestDelay, drone.down_right())

                        elif objLoc_forMovementCalculations[0] < midPlusSafeZoneRad:
                           #-- will call up and left 3 times [comms are usualy unstable so we send the request multiple times]
                           sendCommandsMultipleTimesToInsureDelivery(drone.requestAmount, drone.requestDelay, drone.down_left())

                        else:
                            sendCommandsMultipleTimesToInsureDelivery(drone.requestAmount, drone.requestDelay, drone.down())

            else:
                if objLoc_forMovementCalculations[0] > midPlusSafeZoneRad:
                           #-- will call up and right 3 times [comms are usualy unstable so we send the request multiple times]
                           sendCommandsMultipleTimesToInsureDelivery(drone.requestAmount, drone.requestDelay, drone.right())

                elif objLoc_forMovementCalculations[0] < midPlusSafeZoneRad:
                           #-- will call up and left 3 times [comms are usualy unstable so we send the request multiple times]
                           sendCommandsMultipleTimesToInsureDelivery(drone.requestAmount, drone.requestDelay, drone.left())

       
        #-- this is just the drawing and showing part -> this has nothing to do with the calculation --#

        cv2.circle(image, data.centerPage, safeZoneRad, (255, 0, 0), 2)
		#-- drawlineFromCenterTo obj
        cv2.line(image, data.get__objLoc(), data.centerPage, (0,255,255), 2)
        #-- show the frame to our screen and increment the frame counter
        cv2.imshow("RGB", image)
        count = count + 1
        key = cv2.waitKey(1) & 0xFF
        counter += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(1/30)
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()