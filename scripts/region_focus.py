#!/usr/bin/env python

from __future__ import print_function

import copy

import numpy as np

# ROS
import rospy
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# ===================== Constants. ===========================

IDX_HEIGHT = 0
IDX_WIDTH  = 1

IDX_X = 0
IDX_Y = 1

# TOPIC_SUBSCRIBED = "/xic_stereo/left/image_raw"
TOPIC_SUBSCRIBED = "/xic_stereo/right/image_raw"
TOPIC_PUBLISHED  = "image_regions"
TOPIC_ANNOTATED  = "image_annotated"

FRAME_ID = "region_focus_9"

ROS_NODE_NAME = "region_focus"

# ===================== Functions. ============================

# ==================== Classes. ===========================

class RegionFocusPublisher:
    def __init__(self, ratioTopLeft, regionSize):
        self.cvBridge = CvBridge()

        self.oriImgSize   = [0, 0] # Height, width
        self.ratioTopLeft = ratioTopLeft # [x ratio, y ratio]
        self.regionSize   = regionSize # [Height, width].

        self.cvAssembledImage = np.zeros(\
            (self.regionSize[IDX_HEIGHT]*3, self.regionSize[IDX_WIDTH]*3, 3), dtype=np.uint8 )

        self.msgCount = 0

        self.publisherImage = rospy.Publisher(TOPIC_PUBLISHED,  Image, queue_size = 1)
        self.publisherAnnotated = rospy.Publisher(TOPIC_ANNOTATED, Image,  queue_size = 1)
        self.subsciber = rospy.Subscriber(TOPIC_SUBSCRIBED, Image, self.callback_subscriber)

    def nine_regions(self):
        """Calculates the starting coordinate and size for each image region."""

        coordinateList = []

        # Top left.
        x = (int)( self.oriImgSize[IDX_WIDTH]  * self.ratioTopLeft[IDX_X] )
        y = (int)( self.oriImgSize[IDX_HEIGHT] * self.ratioTopLeft[IDX_Y] )
        coordinateList.append( [x, y] )

        # Top center.
        x = (int)( self.oriImgSize[IDX_WIDTH]  * 0.5 - self.regionSize[IDX_WIDTH] / 2 )
        y = (int)( self.oriImgSize[IDX_HEIGHT] * self.ratioTopLeft[IDX_Y] )        
        coordinateList.append( [x, y] )

        # Top right.
        x = (int)( self.oriImgSize[IDX_WIDTH]  * ( 1.0 - self.ratioTopLeft[IDX_X] ) - self.regionSize[IDX_WIDTH] )
        y = (int)( self.oriImgSize[IDX_HEIGHT] * self.ratioTopLeft[IDX_Y] )
        coordinateList.append( [x, y] )

        # Center left.
        x = (int)( self.oriImgSize[IDX_WIDTH]  * self.ratioTopLeft[IDX_X] )
        y = (int)( self.oriImgSize[IDX_HEIGHT] * 0.5 - self.regionSize[IDX_HEIGHT] / 2 )
        coordinateList.append( [x, y] )

        # Center.
        x = (int)( self.oriImgSize[IDX_WIDTH]  * 0.5 - self.regionSize[IDX_WIDTH]  / 2 )
        y = (int)( self.oriImgSize[IDX_HEIGHT] * 0.5 - self.regionSize[IDX_HEIGHT] / 2 )
        coordinateList.append( [x, y] )

        # Center right.
        x = (int)( self.oriImgSize[IDX_WIDTH]  * (1.0 - self.ratioTopLeft[IDX_X]) - self.regionSize[IDX_WIDTH] )
        y = (int)( self.oriImgSize[IDX_HEIGHT] * 0.5 - self.regionSize[IDX_HEIGHT] / 2 )
        coordinateList.append( [x, y] )

        # Bottom left.
        x = (int)( self.oriImgSize[IDX_WIDTH]  * self.ratioTopLeft[IDX_X] )
        y = (int)( self.oriImgSize[IDX_HEIGHT] * (1.0 - self.ratioTopLeft[IDX_Y]) - self.regionSize[IDX_HEIGHT] )
        coordinateList.append( [x, y] )

        # Bottom center.
        x = (int)( self.oriImgSize[IDX_WIDTH]  * 0.5 - self.regionSize[IDX_WIDTH] / 2 )
        y = (int)( self.oriImgSize[IDX_HEIGHT] * (1.0 - self.ratioTopLeft[IDX_Y]) - self.regionSize[IDX_HEIGHT] )
        coordinateList.append( [x, y] )

        # Bottom right.
        x = (int)( self.oriImgSize[IDX_WIDTH]  * (1.0 - self.ratioTopLeft[IDX_X]) - self.regionSize[IDX_WIDTH]  )
        y = (int)( self.oriImgSize[IDX_HEIGHT] * (1.0 - self.ratioTopLeft[IDX_Y]) - self.regionSize[IDX_HEIGHT] )
        coordinateList.append( [x, y] )

        return coordinateList

    def callback_subscriber(self, data):
        # rospy.loginfo(rospy.get_caller_id() + " Message received.")

        self.msgImages = []

        try:
            # Convert your ROS Image message to OpenCV2
            cvImg = self.cvBridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Get the dimensions of the image.
            self.oriImgSize[IDX_HEIGHT] = cvImg.shape[0]
            self.oriImgSize[IDX_WIDTH]  = cvImg.shape[1]

            coordinateList = self.nine_regions()

            # Sample on the original image.
            
            coorIdx = 0

            for i in range(3):
                for j in range(3):
                    coor = coordinateList[coorIdx]
                    x = coor[IDX_X]
                    y = coor[IDX_Y]

                    idxAssembledImageI = i*self.regionSize[IDX_HEIGHT]
                    idxAssembledImageJ = j*self.regionSize[IDX_WIDTH]

                    self.cvAssembledImage[\
                        idxAssembledImageI:(idxAssembledImageI+self.regionSize[IDX_HEIGHT]), \
                        idxAssembledImageJ:(idxAssembledImageJ+self.regionSize[IDX_WIDTH]), : ] =\
                        cvImg[ y:(y+self.regionSize[IDX_HEIGHT]), x:(x+self.regionSize[IDX_WIDTH]), : ]

                    coorIdx += 1

            coorIdx = 0

            for i in range(3):
                for j in range(3):
                    coor = coordinateList[coorIdx]
                    x = coor[IDX_X]
                    y = coor[IDX_Y]

                    cv2.rectangle(cvImg, (x, y), (x+self.regionSize[IDX_HEIGHT], y+self.regionSize[IDX_WIDTH]), (0, 255, 0), 4)

                    coorIdx += 1

            timeStamp = rospy.Time.now()

            try:
                msgImage              = self.cvBridge.cv2_to_imgmsg(self.cvAssembledImage, "bgr8")
                msgImage.header.seq   = self.msgCount
                msgImage.header.stamp = timeStamp

                self.publisherImage.publish( msgImage )

                msgImageAnnotated              = self.cvBridge.cv2_to_imgmsg(cvImg, "bgr8")
                msgImageAnnotated.header.seq   = self.msgCount
                msgImageAnnotated.header.stamp = timeStamp

                self.publisherAnnotated.publish( msgImageAnnotated )
            except rospy.ROSException, ex:
                # At the very first stages, ROS will complain about not initialized node.
                rospy.loginfo(rospy.get_caller_id() + ex.message)

            self.msgCount += 1

            rospy.loginfo(rospy.get_caller_id() + " Regions published.")

# =================== main. ==========================

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, anonymous = True)
    
    rfp = RegionFocusPublisher([0.1,0.1], [300, 300])

    rospy.spin()
