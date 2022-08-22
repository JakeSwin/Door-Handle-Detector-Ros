#!/usr/bin/python
import image_geometry
import numpy
import ros_numpy
import rospy
import tf

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from door_handle_detector_msgs.msg import BoundingBoxList, BoundingBox
from door_handle_detector_msgs.srv import BoundingBoxService, BoundingBoxServiceRequest, BoundingBoxServiceResponse 

from cv_bridge import CvBridge, CvBridgeError

import cv2

content_type = 'image/jpeg'
headers = {'content-type': content_type}

import requests
import os

def main():
    print("Starting node")
    rospy.init_node('door_handle_detector', anonymous=True)

    service = "get_bounding_box"
    bridge = CvBridge()
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(1.0)

    rospy.wait_for_service(service)
    
    while not rospy.is_shutdown():
        depth_image = rospy.wait_for_message("/xtion/depth_registered/image_raw", Image)
        point_cloud = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        info = rospy.wait_for_message("/xtion/depth_registered/camera_info", CameraInfo)

        proxy = rospy.ServiceProxy(service, BoundingBoxService)
        try:
            service_response = proxy(BoundingBoxServiceRequest())
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            continue
        try:
            bbox = service_response.bounding_boxes.bounding_boxes[0]
        except:
            print("did not recieve bounding box")
            continue

        try:
            cv_image = bridge.imgmsg_to_cv2(depth_image)
        except CvBridgeError as e:
            print(e)
            continue

        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(info)

        x_center = bbox.x_max - ((bbox.x_max - bbox.x_min) / 2)
        y_center = bbox.y_max - ((bbox.y_max - bbox.y_min) / 2)

        if x_center > point_cloud.width or y_center > point_cloud.height or x_center < 0 or y_center < 0:
            continue 

        gen = pc2.read_points(point_cloud, uvs=[(x_center, y_center)])

        for p in gen:
            print("x : %f y: %f z : %f" %(p[0], p[1], p[2]))

        print(point_cloud.width)
        print(point_cloud.height)
        print(point_cloud.row_step)
        print(point_cloud.point_step)
        print(point_cloud.header.stamp)
        print(point_cloud.header.frame_id)

        br.sendTransform((p[0], p[1], p[2]),
                        (0.0, 0.0, 0.0, 1.0),
                        point_cloud.header.stamp,
                        "handle",
                        cam_model.tfFrame())
        rate.sleep()

    print("Shutting down")

if __name__ == '__main__':
    main()
