#!/usr/bin/env python

import argparse
import struct
import sys

import rospy
import rospkg

import intera_interface

def main():
    rospy.init_node("ik_pick_and_place_demo")
    intera_interface.Gripper().close()
    rospy.sleep(0.25)

if __name__ == '__main__':
    sys.exit(main())
