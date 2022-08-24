#!/usr/bin/python
import rospy
import copy
import sys
import tf
import moveit_commander

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import DisplayTrajectory

END_EFFECTOR_LINK = "gripper_grasping_frame"

def wait_for_box(box_name, robot, scene):
    start = rospy.get_time()
    seconds = rospy.get_time()
    timeout = 15 
    while (seconds - start < timeout) and not rospy.is_shutdown():
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        is_known = box_name in scene.get_known_object_names()
        print(is_attached)
        print(is_known)
        if is_attached and is_known:
            print("box is attached")
            return True

        rospy.sleep(0.2)
        seconds = rospy.get_time()
    return False

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("open_door_node", anonymous=True)
    listener = tf.TransformListener()

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    box_pose = PoseStamped()
    box_pose.header.frame_id = "handle"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    
    grasping_group = "gripper"
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(END_EFFECTOR_LINK, box_name, touch_links=touch_links)

    wait_for_box(box_name, robot, scene)

    group_name = "arm_torso"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    planning_frame = move_group.get_planning_frame()
    print("Planning Frame: %s" % planning_frame)

    print("Setting custom end effector")
    move_group.set_end_effector_link(END_EFFECTOR_LINK)

    eef_link = move_group.get_end_effector_link()
    print("End effector link: %s" % eef_link)
    
    group_names = robot.get_group_names()
    print("Available Planning Groups: %s" % group_names)

    print("=== Printing Robot State ===")
    print(robot.get_current_state())
    print("=== Stopped Printing Robot State ===")

    display_trajectory_publisher = rospy.Publisher("/display_planned_path", DisplayTrajectory, queue_size=20)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/base_footprint', '/handle', rospy.Time(0))
            print(trans)
            print(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        waypoints = []
        #start_pose = move_group.get_current_pose().pose
        #waypoints.append(start_pose)

        # pose_goal = PoseStamped()        
        pose_goal = Pose()

        # pose_goal.header.frame_id = "base_footprint"
        pose_goal.position.x = trans[0] - 0.3 
        pose_goal.position.y = trans[1] 
        pose_goal.position.z = trans[2]  

        pose_goal.orientation.x = rot[0]
        pose_goal.orientation.y = rot[1]
        pose_goal.orientation.z = rot[2]
        pose_goal.orientation.w = rot[3] 

        (rot_x, rot_y, rot_z) = euler_from_quaternion(rot)
        
        print(pose_goal)

        waypoints.append(copy.deepcopy(pose_goal))

        pose_goal.position.x = trans[0] - 0.10
        pose_goal.position.z = trans[2] + 0.05

        rot_y += 0.5854
        new_rot = quaternion_from_euler(rot_x, rot_y, rot_z)
        
        pose_goal.orientation.x = new_rot[0]
        pose_goal.orientation.y = new_rot[1]
        pose_goal.orientation.z = new_rot[2]
        pose_goal.orientation.w = new_rot[3] 

        waypoints.append(copy.deepcopy(pose_goal))

        pose_goal.position.x = trans[0] - 0.07
        pose_goal.position.z = trans[2] - 0.1
        
        waypoints.append(copy.deepcopy(pose_goal))

        print("Waypoints:")
        print(waypoints)
        print("planning")
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 2.0, 0.0, avoid_collisions=False)
        print("found_plan")
        print("fraction: %s" % fraction)

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

        #plan.joint_trajectory.points[0].time_from_start = 3
        # move_group.set_pose_target(waypoints[2])
        move_group.set_goal_tolerance(0.05)

        # plan = move_group.retime_trajectory(plan)

        # plan = move_group.go(wait=True)
        if fraction == 1.0:
            move_group.execute(plan, wait=True)

            move_group.stop()

            move_group.clear_pose_targets()
        else:
            print("Fraction was not 1.0, not executing path")
        break
        rospy.sleep(1)
    print("Done")

if __name__ == "__main__":
    main()
