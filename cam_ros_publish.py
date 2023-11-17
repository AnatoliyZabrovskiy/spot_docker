import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import time

class Ros(object):
    def __init__(self):
        # Params
        self.image_rgb = None
        self.image_depth = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(0.2) # 0.2hz = every 5 sec.

        # Subscribers
        self.rgb_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback_rgb)
        self.depth_sub = rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, self.callback_depth)

        # Publishers
        self.pub_rgb = rospy.Publisher('/camera/rgb/image_rect_color_low_fps', Image, queue_size=10)
        self.pub_depth = rospy.Publisher('/camera/depth_registered/hw_registered/image_rect_low_fps', Image, queue_size=10)


    def callback_rgb(self, msg):
        #rospy.loginfo('RGB image received...')
        self.image_rgb = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def callback_depth(self, msg):
        #rospy.loginfo('Depth image received...')
        self.image_depth = self.br.imgmsg_to_cv2(msg)

    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            #br = CvBridge()
            if self.image_rgb is not None:
                self.pub_rgb.publish(self.br.cv2_to_imgmsg(self.image_rgb, "rgb8"))
                print('RGB image published')
            if self.image_depth is not None:
                self.pub_depth.publish(self.br.cv2_to_imgmsg(self.image_depth))
                print('Depth image published')
            #self.loop_rate.sleep()
            time.sleep(6)

if __name__ == '__main__':
    rospy.init_node("rosimage", anonymous=True)
    my_node = Ros()
    my_node.start()