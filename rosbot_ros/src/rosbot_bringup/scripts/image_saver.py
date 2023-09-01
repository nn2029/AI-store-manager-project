#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.counter = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            cv2.imwrite("image_{0:04d}.jpg".format(self.counter), cv_image)
            self.counter += 1

def main():
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
