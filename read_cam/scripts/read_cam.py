#!/usr/bin/env python2
import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
 
 

class image_converter:
 
    def __init__(self):
        self.id=0
        self.cap = cv2.VideoCapture(self.id)
        #read frame or not
        self.ret = False
        #self.image_pub = rospy.Publisher("image_topic",CompressedImage,queue_size=1)
        self.image_pub = rospy.Publisher("image_topic",Image,queue_size=1)
        self.bridge = CvBridge()
        self.show()

        

    def show(self):
        while(1):
        # get a frame
            self.ret, frame = self.cap.read()
            if(self.ret == False):
                self.id = self.id+1
                self.cap = cv2.VideoCapture(self.id)
                self.ret, frame = self.cap.read()
            # show a frame
            frame=cv2.resize(frame, None, fx=0.3, fy=0.3)
            # cv2.imshow("capture", frame)
            #print(frame.shape)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

            try:
                #self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(frame, "jpg"))
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError as e:
                print(e)

    
 
if __name__ == '__main__':
    rospy.init_node('usbcam', anonymous=True)
    rospy.loginfo("Starting usbcam node")
    ic = image_converter()
    cv2.destroyAllWindows()
