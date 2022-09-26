#!/usr/bin/python
import image_geometry
import numpy
import ros_numpy
import rospy
import tf

from tf.transformations import quaternion_from_euler

import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from door_handle_detector_msgs.msg import BoundingBoxList, BoundingBox
from door_handle_detector_msgs.srv import BoundingBoxService, BoundingBoxServiceRequest, BoundingBoxServiceResponse, HandlePoseService, HandlePoseServiceResponse

from cv_bridge import CvBridge, CvBridgeError

import cv2

content_type = 'image/jpeg'
headers = {'content-type': content_type}

import requests
import os

class handle_finder:
    def __init__(self):
        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(1.0)

        rospy.wait_for_service("get_bounding_box")
        self.info = rospy.wait_for_message("/xtion/depth_registered/camera_info", CameraInfo)
        self.proxy = rospy.ServiceProxy("get_bounding_box", BoundingBoxService)
        self.pub = rospy.Publisher("handle_pose", PoseStamped, queue_size=10)
        self.s = rospy.Service("get_handle_pose", HandlePoseService, self.get_handle_pose)

    def get_handle_pose(self, req):
        while not rospy.is_shutdown():
            rospy.sleep(10)
            depth_image = rospy.wait_for_message("/xtion/depth_registered/image_raw", Image)
            point_cloud = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            
            try:
                service_response = self.proxy(BoundingBoxServiceRequest())
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                continue
            try:
                bbox = service_response.bounding_boxes.bounding_boxes[0]
            except:
                rospy.loginfo("did not recieve bounding box")
                continue

            try:
                cv_image = self.bridge.imgmsg_to_cv2(depth_image)
            except CvBridgeError as e:
                print(e)
                continue

            cam_model = image_geometry.PinholeCameraModel()
            cam_model.fromCameraInfo(self.info)

            box_width = (bbox.x_max - bbox.x_min)
            box_height = (bbox.y_max - bbox.y_min)
            x_center = bbox.x_max - (box_width / 2)
            y_center = bbox.y_max - (box_height / 2)

            if x_center > (point_cloud.width / 2):
                x_center -= (box_width / 5)
            else:
                x_center += (box_width / 5)

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

            # quaternion = (0.0, 0.0, 0.0, 1.0)
            # quaternion = quaternion_from_euler(-1.5707, 0, 1.5707)
            q = quaternion_from_euler(1.5707, -1.5707, 1.5707)

            if isinstance(p[0], float) and isinstance(p[1], float) and isinstance(p[2], float):
                my_header = Header(stamp=point_cloud.header.stamp, frame_id=cam_model.tfFrame())
                my_pose = Pose(position=Point(x=p[0], y=p[1], z=p[2]), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
                pose_stamped = PoseStamped(header=my_header, pose=my_pose)

                self.pub.publish(pose_stamped)

                #br.sendTransform((p[0], p[1], p[2]),
                #                quaternion,
                #                point_cloud.header.stamp,
                #                "handle",
                #                cam_model.tfFrame())

                return HandlePoseServiceResponse(pose_stamped)
            else:
                rospy.loginfo("invalid coordinates, skipping to next iteration")

def main():
    print("Starting node")
    rospy.init_node('door_handle_detector', anonymous=True)

    HandleFinder = handle_finder()

    rospy.spin()

if __name__ == '__main__':
    main()
