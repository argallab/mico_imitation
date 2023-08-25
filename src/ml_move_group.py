#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from mico_ml.srv import QueryML, QueryMLResponse, QueryMLRequest
from mico_ml.srv import QueryMLVel, QueryMLVelResponse, QueryMLVelRequest
from mico_ml.srv import Policy, PolicyResponse, PolicyRequest


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "mico_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  # set and get web client current policy
  def set_policy(self, policy):
    rospy.wait_for_service("web_client_node/set_policy")
    rospy.ServiceProxy("web_client_node/set_policy", Policy)(policy)

  def get_policy(self):
    rospy.wait_for_service("web_client_node/get_policy")
    policy = rospy.ServiceProxy("web_client_node/get_policy", Policy)("")
    return policy.output
  
  # sets start pose in web client to current pose
  def set_start_pose(self):
    rospy.wait_for_service("web_client_node/set_start_pose")
    set_start = rospy.ServiceProxy("web_client_node/set_start_pose", QueryMLVel)

    current_pose = self.group.get_current_pose().pose
    result = set_start(current_pose)

    print(result)

  # go to position
  def go_to_position(self):
    rospy.wait_for_service("web_client_node/get_next_position")
    get_next = rospy.ServiceProxy("web_client_node/get_next_position", QueryMLVel)

    current_pose = self.group.get_current_pose().pose
    next = get_next(current_pose).next
    print(next)
    self.group.set_pose_target(next, self.eef_link)

    plan = self.group.go(wait=True)
    self.group.stop()
    self.group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    print(current_pose)

    # print(all_close(test_pose, current_pose, 0.01))

  def go_to_start(self, start):
    self.group.set_position_target(start, self.eef_link)
    plan = self.group.go(wait=True)
    self.group.stop()
    self.group.clear_pose_targets()

  def go_to_start_pose(self, start=None):
    if start is None:
      # get start pose from web
      rospy.wait_for_service("web_client_node/get_start_pose")
      get_start = rospy.ServiceProxy("web_client_node/get_start_pose", QueryMLVel)
      start = get_start(self.group.get_current_pose().pose).next

    self.group.set_pose_target(start, self.eef_link)
    plan = self.group.go(wait=True)
    self.group.stop()
    self.group.clear_pose_targets()

  def query_pose(self, move=False, verbose=False):
    rospy.wait_for_service("web_client_node/get_next_pose")
    get_next = rospy.ServiceProxy("web_client_node/get_next_pose", QueryMLVel)

    # get current pose
    current_pose = self.group.get_current_pose().pose
    if verbose:
      print("current position")
      print(current_pose)

    # query ML for next pose
    next_pose = get_next(current_pose).next
    if verbose:
      print("next position from web ml")
      print(next_pose)

    # normalize orientation quarterion
    next_orientation = self.quat_to_np(next_pose.orientation)
    norm = next_orientation / np.linalg.norm(next_orientation)
    next_pose.orientation = self.np_to_quat(norm)

    if verbose:
      print("next position with norm")
      print(next_pose)

    if move:
      # move
      self.group.set_pose_target(next_pose, self.eef_link)
      plan = self.group.go(wait=True)
      self.group.stop()
      self.group.clear_pose_targets()

      # check pose after
      current_pose = self.group.get_current_pose().pose
      # if verbose:
      print("position after move")
      print(current_pose)
      print(all_close(next_pose, current_pose, 0.01))


  def quat_to_np(self, quat):
    return [quat.x, quat.y, quat.z, quat.w]
  
  def np_to_quat(self, np):
    return geometry_msgs.msg.Quaternion(np[0], np[1], np[2], np[3])

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


def main():
  try:
    m = MoveGroupPythonIntefaceTutorial()
    m.set_policy("bc_policy")
    print("using " + m.get_policy())

    # m.set_start_pose()

    for _ in range(50):
      m.go_to_position()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()