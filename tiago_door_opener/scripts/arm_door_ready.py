#!/usr/bin/python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def main():
    rospy.init_node("arm_door_ready_client")
    client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server()
    print("found server")

    goal = FollowJointTrajectoryGoal()

    goal.trajectory.joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.positions = [1.5, -1.0, -3.0, 2.5, -1.57, -1.0, 1.5]
    trajectory_point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    trajectory_point.time_from_start = rospy.Duration(15.0)
    goal.trajectory.points = [trajectory_point]
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
    client.send_goal(goal)
    print("Sent goal")
    client.wait_for_result(rospy.Duration.from_sec(20.0))
    print("Ready")

if __name__ == "__main__":
    main()
