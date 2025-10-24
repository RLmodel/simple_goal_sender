#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalStatus

def send_goal(x, y, yaw_deg, frame="map", timeout=60.0):
    if not rospy.core.is_initialized():
        rospy.init_node("send_move_base_goal", anonymous=True)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("waiting for move_base action server...")
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("move_base server not available")
        return False

    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg))

    goal = MoveBaseGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = qx
    goal.target_pose.pose.orientation.y = qy
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw

    rospy.loginfo("sending goal: x=%.3f y=%.3f yaw=%.1f° frame=%s", x, y, yaw_deg, frame)
    client.send_goal(goal)

    finished = client.wait_for_result(rospy.Duration(timeout))
    if not finished:
        rospy.logwarn("timeout %.1fs. canceling goal.", timeout)
        client.cancel_goal()
        return False

    state = client.get_state()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("goal reached")
        return True
    rospy.logwarn("goal failed. state=%d", state)
    return False

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("usage: rosrun movebase_goal_sender send_goal.py <x> <y> <yaw_deg> [frame] [timeout_sec]")
        print("example: rosrun movebase_goal_sender send_goal.py 1.0 2.0 90 map 60")
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw_deg = float(sys.argv[3])
    frame = sys.argv[4] if len(sys.argv) > 4 else "map"
    timeout = float(sys.argv[5]) if len(sys.argv) > 5 else 60.0

    ok = send_goal(x, y, yaw_deg, frame, timeout)
    sys.exit(0 if ok else 1)
