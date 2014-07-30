#!/usr/bin/env python

# Copyright (c) 2014, Michael E. Ferguson
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# This moves the UBR-1 arm from end to end, at several velocities/accelerations
# To capture bagfile, use:
#
#   rosbag record tf arm_controller/follow_joint_trajectory/feedback robot_debug
#

import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint",
               "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

if __name__=="__main__":
    rospy.init_node("ubr1_arm_trajectory")

    # Connect to arm controller for playback
    rospy.loginfo("Waiting for arm_controller...")
    client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server()
    rospy.loginfo("...connected.")

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    # First point, get arm in place
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = [0.0 for i in trajectory.joint_names]
    trajectory.points[0].positions[0] = -1.39
    trajectory.points[0].velocities = [0.0 for i in trajectory.joint_names]
    trajectory.points[0].accelerations = [0.0 for i in trajectory.joint_names]
    trajectory.points[0].time_from_start = rospy.Duration(5.0)

    # Slow to other side
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[1].positions = [0.0 for i in trajectory.joint_names]
    trajectory.points[1].positions[0] = 1.39
    trajectory.points[1].velocities = [0.0 for i in trajectory.joint_names]
    trajectory.points[1].accelerations = [0.0 for i in trajectory.joint_names]
    trajectory.points[1].time_from_start = rospy.Duration(10.0)

    # Back again faster
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[2].positions = [0.0 for i in trajectory.joint_names]
    trajectory.points[2].positions[0] = -1.39
    trajectory.points[2].velocities = [0.0 for i in trajectory.joint_names]
    trajectory.points[2].accelerations = [0.0 for i in trajectory.joint_names]
    trajectory.points[2].time_from_start = rospy.Duration(12.5)

    # Back again faster
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[3].positions = [0.0 for i in trajectory.joint_names]
    trajectory.points[3].positions[0] = 1.39
    trajectory.points[3].velocities = [0.0 for i in trajectory.joint_names]
    trajectory.points[3].accelerations = [0.0 for i in trajectory.joint_names]
    trajectory.points[3].time_from_start = rospy.Duration(13.5)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)

    try:
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(15.0))
    except:
        rospy.logerr("Failed!")

