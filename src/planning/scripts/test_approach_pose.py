#!/usr/bin/env python

# Copyright (c) 2015-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Sawyer SDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import intera_interface

class PickAndPlace(object):
    def __init__(self, limb="right", hover_distance = 0.15, tip_name="right_gripper_tip"):
        self._limb_name = limb # string
        self._tip_name = tip_name # string
        self._hover_distance = hover_distance # in meters
        self._limb = intera_interface.Limb(limb)
        self._gripper = intera_interface.Gripper()
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
      
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)

    def _guarded_move_to_joint_position(self, joint_angles, timeout=10.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self._limb.ik_request(approach, self._tip_name)
        self._limb.set_joint_position_speed(0.001)
        self._guarded_move_to_joint_position(joint_angles)
        self._limb.set_joint_position_speed(0.1)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        self._servo_to_pose(ik_pose)

    def _servo_to_pose(self, pose, time=1.5, steps=150.0):
        ''' An *incredibly simple* linearly-interpolated Cartesian move '''
        r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
        current_pose = self._limb.endpoint_pose()
        ik_delta = Pose()
        ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps
        ik_delta.position.y = (current_pose['position'].y - pose.position.y) / steps
        ik_delta.position.z = (current_pose['position'].z - pose.position.z) / steps
        ik_delta.orientation.x = (current_pose['orientation'].x - pose.orientation.x) / steps
        ik_delta.orientation.y = (current_pose['orientation'].y - pose.orientation.y) / steps
        ik_delta.orientation.z = (current_pose['orientation'].z - pose.orientation.z) / steps
        ik_delta.orientation.w = (current_pose['orientation'].w - pose.orientation.w) / steps
        # ik_delta.orientation.x = 0
        # ik_delta.orientation.y = 0
        # ik_delta.orientation.z = 0
        # ik_delta.orientation.w = 0

        for d in range(int(steps), -1, -1):
            if rospy.is_shutdown():
                return
            ik_step = Pose()
            ik_step.position.x = d*ik_delta.position.x + pose.position.x
            ik_step.position.y = d*ik_delta.position.y + pose.position.y
            ik_step.position.z = d*ik_delta.position.z + pose.position.z
            ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
            ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
            ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
            ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
            # print("step_" + str(d) + str(ik_step))
            joint_angles = self._limb.ik_request(pose = ik_step, end_point= self._tip_name)
            # print(joint_angles)
            if joint_angles:
                # self._limb.set_joint_position_speed(0.001)
                # self._guarded_move_to_joint_position(joint_angles)
                # self._limb.set_joint_position_speed(0.1)
                self._limb.set_joint_positions(joint_angles)
            else:
                rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            r.sleep()
        rospy.sleep(1.0)

    def pick(self, pose):
        if rospy.is_shutdown():
            return
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        if rospy.is_shutdown():
            return
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        if rospy.is_shutdown():
            return
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        if rospy.is_shutdown():
            return
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def main():
    """SDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Sawyer will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame

    limb = 'right'
    hover_distance = 0.15 # meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4':  -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}
    pnp = PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.00142460053167,
                             y=0.999994209902,
                             z=-0.00177030764765,
                             w=0.00253311793936)

    pick_pose = list()             
    pick_pose.append(Pose( position=Point(x=0.55, y=-0.305, z=-0.129), orientation=overhead_orientation))
    pick_pose.append(Pose( position=Point(x=0.75, y=-0.305, z=-0.129), orientation=overhead_orientation))
    pick_pose.append(Pose( position=Point(x=0.55, y=0.085, z=-0.129), orientation=overhead_orientation))
    pick_pose.append(Pose( position=Point(x=0.75, y=0.085, z=-0.129), orientation=overhead_orientation))
    pick_pose.append(Pose( position=Point(x=0.55, y=0.385, z=-0.129), orientation=overhead_orientation))
    pick_pose.append(Pose( position=Point(x=0.75, y=0.385, z=-0.129), orientation=overhead_orientation))

    place_pose = list()
    place_pose.append(Pose( position=Point(x=-0.1, y=0.5, z=-0.129), orientation=overhead_orientation))
    place_pose.append(Pose( position=Point(x=-0.3, y=0.5, z=-0.129), orientation=overhead_orientation))
    place_pose.append(Pose( position=Point(x=-0.4, y=0.5, z=-0.129), orientation=overhead_orientation))
    
    # Move to the desired starting angles
    print("Running. Ctrl-c to quit")
    # pnp.move_to_start(starting_joint_angles)
    # pnp._approach(block_poses[0])
    #     # servo above pose
    #     # servo to pose
    # pnp._servo_to_pose(block_poses[0])
    #     # close gripper
    # pnp.gripper_close()
    # pnp._retract()
    pnp.pick(pick_pose[0])
    # pnp.place(place_pose[0])
    pnp.place(pick_pose[1])
    pnp.pick(pick_pose[2])
    # pnp.place(place_pose[1])
    pnp.place(pick_pose[3])
    pnp.pick(pick_pose[4])
    # pnp.place(place_pose[2])
    pnp.place(pick_pose[5])
if __name__ == '__main__':
    sys.exit(main())
