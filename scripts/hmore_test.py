#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# hmore_test.py : simple test of various Hmore Face capabilites
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2024-2025 Etaoin Systems
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# =========================================================================

import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point

# simple test of various Hmore Face capabilites
class HmoreTest:
  def __init__(self):
    rospy.init_node('hmore_test')
    self.gaze_pub  = rospy.Publisher('gaze',  Point,  queue_size=10)
    self.stare_pub = rospy.Publisher('stare', Bool,   queue_size=10)
    self.expr_pub  = rospy.Publisher('expr',  Point,  queue_size=10)
    self.speak_pub = rospy.Publisher('speak', String, queue_size=10)
    rospy.sleep(1)           # wait for connections


  # short sequence of commands used for video demo
  def Demo(self):
    
    # reset face and wait a while
    neutral = Point(0.0, 0.0, 0.0)
    self.expr_pub.publish(neutral)
    green = Bool(False)
    self.stare_pub.publish(green)
    rospy.sleep(10)

    # look around
    dir = Point(-35.0, 0.0, 45.0)
    self.gaze_pub.publish(dir)
    rospy.sleep(3)
    dir = Point(0.0, 30.0, 45.0)
    self.gaze_pub.publish(dir)
    rospy.sleep(3)
    dir = Point(0.0, 0.0, 45.0)
    self.gaze_pub.publish(dir)
    rospy.sleep(2)

    # say something and smile
    blue = Bool(True)
    self.stare_pub.publish(blue)
    rospy.sleep(1)
    greet = String('hello there human, can I help you with something?')
    self.speak_pub.publish(greet)
    rospy.sleep(3)
    happy = Point(1.0, 0.0, 3.0)
    self.expr_pub.publish(happy)


# =========================================================================

# send face commands then exit
# first start face: roslaunch hmore_face hmore_face.launch
# then run via SSH: rosrun hmore_test hmore_test.py 
if __name__ == '__main__':
  node = HmoreTest()
  node.Demo()


