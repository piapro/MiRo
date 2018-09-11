#!/usr/bin/env python
#	@file
#	@section COPYRIGHT
#   Copyright (C) 2018 Aoheng MA 888788 The Universiry of Melbourne
#	Copyright (C) 2017 Consequential Robotics (CqR)
#
#	@section AUTHOR
#   Aoheng MA

#	Consequential Robotics http://consequentialrobotics.com
#
#	@section LICENSE
#	MIT License
#   Permission is hereby granted, free of charge, to any person obtaining a copy
#   of this software and associated documentation files (the "Software"), to deal
#   in the Software without restriction, including without limitation the rights
#   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the Software is
#   furnished to do so, subject to the following conditions:

#   The above copyright notice and this permission notice shall be included in all
#   copies or substantial portions of the Software.

#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#   SOFTWARE.
#
#   The MiRo SDK and relative libraries are provided by Consequential Robotics (CqR)
#

################################################################################


################################################################################
#For User Interface Implenmentation
################################################################################
'''
#import gi
#gi.require_version('Gtk', '3.0')
#from gi.repository import Gtk,GObject,Gdk,GLib,GdkPixbuf
#GObject.threads_init()
#Gdk.threads_init()
'''
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
from geometry_msgs.msg import Twist

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

import threading
import sys
from miro_constants import miro
from random import randint

import math
import cv2
import numpy
import copy
import time
#INTERPTYPE = GdkPixbuf.InterpType.NEAREST
#INTERPTYPE = GdkPixbuf.InterpType.BILINEAR

################################################################
def fmt(x, f):
    s = ""
    x = bytearray(x)
    for i in range(0, len(x)):
        if not i == 0:
            s = s + ", "
        s = s + f.format(x[i])
    return s

def hex2(x):
    return "{0:#04x}".format(x)

def hex4(x):
    return "{0:#06x}".format(x)

def hex8(x):
    return "{0:#010x}".format(x)

def flt3(x):
    return "{0:.3f}".format(x)

def error(msg):
    print(msg)
    sys.exit(0)

def usage():
    print """
MiRo ROS Off-Board Control
Usage:
    Bash Command:
    miro_ros_client.py robot=<robot_name>

    To run the client off-board the option "robot" must be specified.

Options:
    robot=<robot_name>
        Typically the <robot_name> starts from <rob01> by default,
        you can specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        This argument must be specified when using MiRo Off-board.
    """
    sys.exit(0)

################################################################

class miro_ros_client:

    def callback_platform_sensors(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_sensors = object

        # send downstream command, ignoring upstream data
        q = platform_control()

        # timing
        sync_rate = 50
        period = 2 * sync_rate # two seconds per period
        z = self.count / period

        # advance pattern
        if not z == self.z_bak:
            self.z_bak = z

            # create body_vel for next pattern segment
            self.body_vel = Twist()
            if self.drive_pattern == "square":
                # square dance
                if z == 0 or z == 2:
                    print "turn left"
                    self.body_vel.angular.z = +0.7854
                if z == 1 or z == 3:
                    print "drive forward"
                    self.body_vel.linear.x = +200
            else:
                # do-si-do
                if z == 0:
                    print "turn left"
                    self.body_vel.angular.z = +1.5708
                if z == 1:
                    print "drive forward"
                    self.body_vel.linear.x = +200
                if z == 2:
                    print "turn right"
                    self.body_vel.angular.z = -1.5708
                if z == 3:
                    print "drive forward"
                    self.body_vel.linear.x = +200

        #we can modify this part to make miro avoid objectsself.
        #detect objects ahead,reverse
        #detect objects right, turn left
        #detect objects left, turn right


		# point cameras down
		#q.body_config[1] = 1.0
		#q.body_config_speed[1] = miro.MIRO_P2U_W_LEAN_SPEED_INF

        # publish
        q.body_vel = self.body_vel
        self.pub_platform_control.publish(q)

        # count
        self.count = self.count + 1
        if self.count == 400:
            self.count = 0 #modigy this part

    def callback_platform_state(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_state = object

    def callback_platform_mics(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_mics = object #maybe control Mic to detect voice

    def callback_core_state(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.core_state = object

    def loop(self):
        while True:
            if rospy.core.is_shutdown():
                break
            time.sleep(1)
            print "tick"
            if rospy.core.is_shutdown():
                break
            time.sleep(1)
            print "tock"

    def __init__(self):

        # report
        print("initialising...")
        print(sys.version)

        # default data
        self.platform_sensors = None
        self.platform_state = None
        self.platform_mics = None
        self.core_state = None

        # no arguments gives usage
        if len(sys.argv) == 1:
            usage()

        # options
        self.robot_name = ""
        self.drive_pattern = ""

        # handle args
        for arg in sys.argv[1:]:
            f = arg.find('=')
            if f == -1:
                key = arg
                val = ""
            else:
                key = arg[:f]
                val = arg[f+1:]
            if key == "robot":
                self.robot_name = val
            elif key == "drive":
                self.drive_pattern = val
            else:
                error("argument not recognised \"" + arg + "\"")

        # check we got at least one
        if len(self.robot_name) == 0:
            error("argument \"robot\" must be specified")

        # pattern
        self.count = 0
        self.z_bak = -1
        self.body_vel = None

        # set inactive
        self.active = False

        # topic root
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

        # publish
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control",
                    platform_control, queue_size=0)
        self.pub_core_control = rospy.Publisher(topic_root + "/core/control",
                    core_control, queue_size=0)
        self.pub_core_config = rospy.Publisher(topic_root + "/core/config",
                    core_config, queue_size=0)
        self.pub_bridge_config = rospy.Publisher(topic_root + "/bridge/config",
                    bridge_config, queue_size=0)
        self.pub_bridge_stream = rospy.Publisher(topic_root + "/bridge/stream",
                    bridge_stream, queue_size=0)
        self.pub_platform_config = rospy.Publisher(topic_root + "/platform/config",
                    platform_config, queue_size=0)

        # subscribe
        self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors",
                platform_sensors, self.callback_platform_sensors)
        self.sub_state = rospy.Subscriber(topic_root + "/platform/state",
                platform_state, self.callback_platform_state)
        self.sub_mics = rospy.Subscriber(topic_root + "/platform/mics",
                platform_mics, self.callback_platform_mics)
        self.sub_core_state = rospy.Subscriber(topic_root + "/core/state",
                core_state, self.callback_core_state)

        # set active
        self.active = True


def welcomeMessage :
    print("Welcome to MiRo ! ")
    print("System Initializing ...")

if __name__ == "__main__":
    welcomeMessage()
    rospy.init_node("miro_ros_client_py", anonymous=True)
    main = miro_ros_client()
    main.loop()
