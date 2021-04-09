import numpy as np 
import cv2
from scipy.spatial import distance as dist
from collections import OrderedDict

## Ros libraries
import roslib
import rospy

#class color:

    #Define upper and lower bounds for each color BGR
#    lowerBlack = [0, 0, 0]
#    upperBlack = [5, 50, 50]
#    upperRed = [5, 255, 255]
#    lowerRed = [0, 50, 50]
#    lowerYellow = [25, 50, 50]
#    lowerGreen = [50, 50, 50]
#    upperYellow = [35, 255, 255]
#    lowerBlue = [100, 50, 50]
#    upperGreen = [70, 255, 255]
#    upperBlue = [130, 255, 255]
#    lowerMagenta = [125, 50, 50]
#    upperMagenta = [150, 255, 255]

class ColorLabeler:
    def __init__(self):
        # Mean Color Dictionary RGB
        colors = OrderedDict({
            #"avBlack" : (25, 25, 2.5),
            #"avRed" : (152.5, 152.5, 2.5),
            #"avYellow": (152.5, 152.5, 30),
            #"avGreen": (152.5, 152.5, 60),
            #"avBlue": (152.5, 152.5, 115),
            #"avMagenta":  (152.5, 152.5, 137.5),
            "red":(255, 0, 0),
            "green": (0, 255, 0),
            "blue":(0, 0, 255),
            "black":(0,0,0),
            "yellow":(255, 255, 0),
            "magenta":(255, 0, 255),
            # 
            # ---
            #"UpperYellow":(255, 255, 35),
            #"LowerYellow":(50, 50, 25),
            #"UpperGreen":(255, 255, 70),
            #"LowerGreen":(50, 50, 50),
            #"UpperRed": (255, 255, 5),
            #"LowerRed": (50, 50, 0),
            #"LowerBlue": (50, 50, 100),
            #"UpperBlue": (255, 255, 130),
            #"LowerMagenta": (50, 50, 125),
            #"UpperMagenta": (255, 255, 150),
            #"UpperBlack": (50, 50, 5),
            # ---
            #"LowerBlue": (25, 0, 90),
            #"UpperBlue": (75, 0, 255),
            #"LowerGreen":(20, 100, 0),
            #"UpperGreen": (50, 255, 0),
            #"LowerMagenta": (90, 0, 90),
            #"LowerYellow": (80, 100, 5),
            #"UpperYellow": (240, 255, 20),


        })
        # allocate memory for the L*a*b* image, then initialize
		# the color names list
        self.lab = np.zeros((len(colors), 1, 3), dtype="uint8")
        self.colorNames = []

        # loop over the colors dictionary
        for (i, (name, rgb)) in enumerate(colors.items()):
        # update the L*a*b* array and the color names list
            self.lab[i] = rgb
            self.colorNames.append(name)
        # convert the L*a*b* array from the RGB color space
        # to L*a*b*
        #self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

        self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2HSV)

    def label(self, image, center, radius):
        # create a mask for the ball contour
        # then compute the avarage value for the masked region
        mask = np.zeros((image.shape[0],image.shape[1]), np.uint8)
        cv2.circle(mask, center, radius, 255, -1)
        kernel = np.ones((9, 9), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        #masked = cv2.bitwise_and(image, image, mask=mask)

        #cv2.imshow("mask", mask)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
       
        #cv2.imshow("imagelab", image)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

        mean = cv2.mean(image, mask=mask)[:3]

        #print("mean :", mean)

        # initialize the minimum distance found thus far
        minDist = (np.inf, None)
        
        # loop over the known L*a*b* color values
        for (i, row) in enumerate(self.lab):
            # compute the distance between the current L*a*b*
            # color value and the mean of the image
            d = dist.euclidean(row[0], mean)
            # if the distance is smaller than the current distance
            # then update the bookkeeping variable
            if d < minDist[0]:
                minDist = (d, i)
        return self.colorNames[minDist[1]]
    
    def generateBoundaries(self, color):
        # colorspace BGR
        if color == "black":
            lowerBound = np.array([0, 0, 0])
            upperBound = np.array([5, 50, 50])
        elif color == "red":
            lowerBound = np.array([0, 50, 50])
            upperBound = np.array([5, 255, 255])
        elif color == "yellow":
            lowerBound = np.array([25 ,50 ,50])
            upperBound = np.array([35, 255, 255])
        elif color == "green":
            lowerBound = np.array([50 ,50 ,50])
            upperBound = np.array([70, 255, 255])
        elif color == "blue":
            lowerBound = np.array([100 ,50 ,50])
            upperBound = np.array([130, 255, 255])
        elif color == "magenta":
            lowerBound = np.array([125 ,50 ,50])
            upperBound = np.array([150, 255, 255])
        else:
            #print("NO COLOR MATCHED")
            lowerBound = None
            upperBound = None

        return lowerBound, upperBound
