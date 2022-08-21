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

def pixelTo3DPoint(point_cloud, u, v):
    width = point_cloud.width
    height= point_cloud.height

    arrayPosition = v*point_cloud.row_step + u*point_cloud.point_step

    arrayPosX = arrayPosition + point_cloud.fields[0].offset
    arrayPosY = arrayPosition + point_cloud.fields[1].offset
    arrayPosZ = arrayPosition + point_cloud.fields[2].offset

    X = point_cloud.data[arrayPosX]
    Y = point_cloud.data[arrayPosY]
    Z = point_cloud.data[arrayPosZ]

    return X, Y, Z

def main():
    print("Starting node")
    rospy.init_node('door_handle_detector', anonymous=True)

    service = "get_bounding_box"
    bridge = CvBridge()
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(4.0)

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
        try:
            bbox = service_response.bounding_boxes.bounding_boxes[0]
        except:
            print("did not recieve bounding box")
            continue

        try:
            cv_image = bridge.imgmsg_to_cv2(depth_image)
        except CvBridgeError as e:
            print(e)

        x_center = (bbox.x_max - bbox.x_min)
        y_center = (bbox.y_max - bbox.y_min)

        array = ros_numpy.point_cloud2.pointcloud2_to_array(point_cloud)
        xyz_array = ros_numpy.point_cloud2.get_xyz_points(array)

        print(type(cv_image))
        print(cv_image[x_center][y_center])

        print(type(xyz_array))
        #print(xyz_array)

        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(info)
        ray = numpy.array(cam_model.projectPixelTo3dRay((x_center, y_center)))
        point3d = ray * cv_image[x_center][y_center] 

        pointX, pointY, pointZ = pixelTo3DPoint(point_cloud, x_center, y_center)
        print("Point3d: ")
        print(point3d)
        
        print("PixelTo3DPoint Value:")
        print(pointX)
        print(pointY)
        print(pointZ)
        
        gen = pc2.read_points(point_cloud, uvs=[(x_center, y_center)])
        print(gen)
        for p in gen:
            print("x : %f y: %f z : %f" %(p[0], p[1], p[2]))
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
