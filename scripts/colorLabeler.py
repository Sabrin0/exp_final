"""! 
    @brief Module of the node BallDetection.
    After a pre-processing phase, due to a simple algorithm it recognizes the mean color of the ball detected. This is done by computing 
    the distance between the color values stored in the dictionary and the mean of the circle image. The color with 
    the smallest distance is returned.
    Moreover the method `generateBoundaries` returns to the BallDetection node the lower and the upper mask for the 
    ball found. 
"""
            
import numpy as np 
import cv2
from scipy.spatial import distance as dist
from collections import OrderedDict

## Ros libraries
import roslib
import rospy

class ColorLabeler:
    def __init__(self):
        """!
            @brief The construct. Initialization of the dictionaries and the labeler

            @param self.ball: `dict`, The keys represent the color of the balls and the corresponing value 
                is a `Bool`: if the specific ball has been detected the flag is set to `True`, otherwise it's ´False´
            
            @param colors: `dict`, it stores all the possible color with the corresponding RGB values 

            @param self.lab: `np.array`, memory allocation for the image 
        """

        ##flag for ball detection, if true the ball is ignored 
        self.ball = {
            "red": False,
            "green": False,
            "blue": False,
            "black": False,
            "yellow": False,
            "magenta": False
        }

        ## Mean Color Dictionary RGB
        colors = OrderedDict({

            "red":(255, 0, 0),
            "green": (0, 255, 0),
            "blue":(0, 0, 255),
            "black":(0,0,0),
            "yellow":(255, 255, 0),
            "magenta":(255, 0, 255),
 
        })

        # allocate memory for the image 
        self.lab = np.zeros((len(colors), 1, 3), dtype="uint8")
        
        # initialize the color names list
        self.colorNames = []

        # loop over the colors dictionary
        for (i, (name, rgb)) in enumerate(colors.items()):
        # update the hsv array and the color names list
            self.lab[i] = rgb
            self.colorNames.append(name)

        # Conversion from RGB to HSV colorspace
        self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2HSV)

    def label(self, image, center, radius):
        """!
            @brief Method of the class colorLabeler. 
            It creates a mask for the ball contour then compute avarage value for the masked region region.
            Then thoug a loop over the known hsv color values the distance between these ones and the mean of the
            masked image is calculated. The know hsv color values with the smaller distance is returned.

            @param image: image to be processed
            @param center: the center of the circle in the image circumscribing the ball
            @param radius: the radius of the circle in the image circumscribing the ball

            @return self.colorName[minDist[1]]: the color found 
        """
        
        mask = np.zeros((image.shape[0],image.shape[1]), np.uint8)
        cv2.circle(mask, center, radius, 255, -1)
        kernel = np.ones((9, 9), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mean = cv2.mean(image, mask=mask)[:3]

        # initialize the minimum distance found thus far
        minDist = (np.inf, None)
        
        # loop over the known hsv color values
        for (i, row) in enumerate(self.lab):
            d = dist.euclidean(row[0], mean)
            if d < minDist[0]:
                minDist = (d, i)
        
        return self.colorNames[minDist[1]]
    
    def generateBoundaries(self, color):
        """! @brief Method of the class colorLabeler.
            Given the color found, it returns the corresponding lower and the upper mask. If the color doesn't
            match any one in the dictionary the method return `None` for both the masks. 

            @param color: The ball color detected in the image

            @return loweBound, upperBound 
        """
        
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
