#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image
from door_handle_detector_msgs.msg import BoundingBoxList, BoundingBox
from door_handle_detector_msgs.srv import BoundingBoxService, BoundingBoxServiceResponse

from cv_bridge import CvBridge, CvBridgeError

import cv2

content_type = 'image/jpeg'
headers = {'content-type': content_type}

import requests
import os

class door_handle_detector:
    def __init__(self):
        self.image_topic = "/xtion/rgb/image_rect_color"
        self.image_pub = rospy.Publisher("handle_bounding_boxes", BoundingBoxList, queue_size=3)
        self.bridge = CvBridge()
        self.service = rospy.Service('get_bounding_box', BoundingBoxService, self.callback)

    def callback(self, data):
        data = rospy.wait_for_message(self.image_topic, Image)
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
        scores = res.json()["preds"]["scores"]

        bbox_msg_list = list()
        for i, bbox in enumerate(bboxes):
            bbox_msg_list.append(BoundingBox(int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]), float(scores[i])))
        try:
            self.image_pub.publish(BoundingBoxList(bbox_msg_list))
        except CvBridgeError as e:
            print(e)
        return BoundingBoxServiceResponse(BoundingBoxList(bbox_msg_list))

def main():
    dhd = door_handle_detector()
    rospy.init_node('door_handle_detector', anonymous=True)

    try:
        rospy.spin()
    except:
        print("Shutting down")

if __name__ == '__main__':
    main()
