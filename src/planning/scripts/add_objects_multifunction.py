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
import random
import time
random.seed()

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    SetModelState
)
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)


import intera_interface

model_path = rospkg.RosPack().get_path('planning')+"/models/"
# Load Table SDF
table_xml = ''
with open (model_path + "cafe_table/model.sdf", "r") as table_file:
    table_xml=table_file.read().replace('\n', '')

# Load Tray URDF
tray_red_xml = ''
with open (model_path + "tray/tray_red.urdf", "r") as tray_file: tray_red_xml=tray_file.read().replace('\n', '')
tray_blue_xml = ''
with open (model_path + "tray/tray_blue.urdf", "r") as tray_file: tray_blue_xml=tray_file.read().replace('\n', '')
tray_green_xml = ''
with open (model_path + "tray/tray_green.urdf", "r") as tray_file: tray_green_xml=tray_file.read().replace('\n', '')

# Load Block URDF
list_block_colour=['block_red', 'block_blue', 'block_green']
block_xml=[]
for d in range(3):
    with open (model_path + "block/"+list_block_colour[d]+".urdf", "r") as block_file:
        block_xml.append(block_file.read().replace('\n', ''))

#Define 6 hard_coded pose
pose=[]
pose.append(Pose(position=Point(x=0.5225, y=-0.2265, z=0.7725)))
# pose.append(Pose(position=Point(x=0.7225, y=-0.2265, z=0.7725)))

pose.append(Pose(position=Point(x=0.5225, y=0.0635, z=0.7725)))
# pose.append(Pose(position=Point(x=0.7225, y=0.0635, z=0.7725)))

pose.append(Pose(position=Point(x=0.5225, y=0.3635, z=0.7725)))
# pose.append(Pose(position=Point(x=0.7225, y=0.3635, z=0.7725)))
pose_range = range(len(pose))

wait_pose = Pose(position=Point(x=0.8225, y=0.1, z=0.7725))

def load_gazebo_table(table_reference_frame="world"):
    print("adding table")
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table_1", table_xml, "/",
                             Pose(position=Point(x=0.75, y=0.1, z=0.0)), table_reference_frame)
        resp_sdf = spawn_sdf("cafe_table_2", table_xml, "/",
                             Pose(position=Point(x=-0.2, y=0.75, z=0.0)), table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_gazebo_tray():
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("tray_1" , tray_red_xml, "/", Pose(position=Point(x=0.1,  y=0.6, z=0.7725)) , "world")
        resp_urdf = spawn_urdf("tray_2" , tray_blue_xml, "/", Pose(position=Point(x=-0.2, y=0.6, z=0.7725)) , "world")
        resp_urdf = spawn_urdf("tray_3" , tray_green_xml, "/", Pose(position=Point(x=-0.5, y=0.6, z=0.7725)) , "world")

    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def load_gazebo_block(block_pose, block_name, block_xml, block_reference_frame="world"):
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf( block_name
                                , block_xml, "/",
                              block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def spam_gazebo_block(block_pose, block_reference_frame="world"):
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf( "block_" + str(time.time())
                                , block_xml[0], "/",
                              block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def delete_gazebo_models(list_delete):
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        for d in list_delete:
            resp_delete = delete_model(d)

        # resp_delete = delete_model("cafe_table")
        # resp_delete = delete_model("block_red")
        # resp_delete = delete_model("block_green")
        # resp_delete = delete_model("block_blue")

    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))

# need test here
def shuffle_gazebo_colour_block():
    try:
        rospy.wait_for_service('/gazebo/set_model_state')
        random.shuffle(pose_range)
        state_msg = ModelState()
        for d in range(3):
            
            state_msg.model_name = list_block_colour[d]
            state_msg.pose = pose[pose_range[d]]
            
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print("Shuffle Model service call failed: {0}".format(e))

def reset_gazebo_colour_block():
    try:
        rospy.wait_for_service('/gazebo/set_model_state')
        state_msg = ModelState()
        for d in range(3):
            
            state_msg.model_name = list_block_colour[d]
            state_msg.pose = pose[d]
            
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print("Shuffle Model service call failed: {0}".format(e))

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
    rospy.init_node("add_object")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    if (rospy.get_param('~action') == "add"):
        load_gazebo_table()
        load_gazebo_tray()
        load_gazebo_block(pose[0], list_block_colour[0], block_xml[0])
        load_gazebo_block(pose[1], list_block_colour[1], block_xml[1])
        load_gazebo_block(pose[2], list_block_colour[2], block_xml[2])
    elif (rospy.get_param('~action') == "destroy"):
        delete_gazebo_models(["cafe_table_1", "cafe_table_2"])
        delete_gazebo_models(["tray_1", "tray_2", "tray_3"])
        delete_gazebo_models(list_block_colour)
    elif (rospy.get_param('~action') == "shuffle"):
        shuffle_gazebo_colour_block()
    elif (rospy.get_param('~action') == "reset"):
        reset_gazebo_colour_block()
    elif (rospy.get_param('~action') == "test"):
        load_gazebo_table(Pose(position=Point(x=0.75, y=0.0, z=0.0)))
        for d in range(len(pose)):
            spam_gazebo_block(pose[d])
    # Remove models from the scene on shutdown

if __name__ == '__main__':
    sys.exit(main())
