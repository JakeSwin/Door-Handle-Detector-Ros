#!/usr/bin/python
import rospy
import sys
import tf
import moveit_commander

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("open_door_node", anonymous=True)
    listener = tf.TransformListener()

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm_torso"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    planning_frame = move_group.get_planning_frame()
    print("Planning Frame: %s" % planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("End effector link: %s" % eef_link)

    group_names = robot.get_group_names()
    print("Available Planning Groups: %s" % group_names)

    print("=== Printing Robot State ===")
    print(robot.get_current_state())
    print("=== Stopped Printing Robot State ===")

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/base_footprint', '/handle', rospy.Time(0))
            print(trans)
            print(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue



        rospy.sleep(1)
    print("Done")

if __name__ == "__main__":
    main()
