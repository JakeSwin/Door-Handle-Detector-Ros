#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import cv2

content_type = 'image/jpeg'
headers = {'content-type': content_type}

import requests
import os
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

script_dir = os.path.dirname(os.path.realpath(__file__))
rel_path = 'camera_image.jpg' 
abs_file_path = script_dir + "/" + rel_path 

class door_handle_detector:
    def __init__(self):
        self.image_pub = rospy.Publisher("hande_bounding_box_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_rect_color", Image, self.callback, queue_size=1, buff_size=2058)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        _, encoded_image = cv2.imencode('.jpg', cv_image)
        encoded_image = encoded_image.tostring()
        
        files={'file': ('image.jpg', encoded_image, content_type)}
        res = requests.post('http://192.168.1.230:8000/upload', files=files)

        print("Status Code: " + str(res.status_code))
        print("Server Message: " + res.json()["message"])
        print(res.json()["preds"])

        bboxes = res.json()["preds"]["bboxes"]

        for bbox in bboxes:
            cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 0, 255), 1)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main():
    dhd = door_handle_detector()
    rospy.init_node('door_handle_detector', anonymous=True)

    try:
        rospy.spin()
    except:
        print("Shutting down")

    #rospy.Subscriber(image_topic, Image, image_callback)
#    while not rospy.is_shutdown():
#        msg = rospy.wait_for_message(image_topic, Image)
#        image_callback(msg)
#        rospy.sleep(3)

if __name__ == '__main__':
    main()
