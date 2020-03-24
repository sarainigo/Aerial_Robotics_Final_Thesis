#!/usr/bin/env python

import rospy
import os
import subprocess
import time
import signal


print("LAUNCH SPHINX")
cmd_sphinx = "sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/airborne.drone"
pro_sphinx = subprocess.Popen(cmd_sphinx, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
time.sleep(10)
print("LAUNCH CLIENT")
cmd_roslaunch = "roslaunch control_parrot control_parrot.launch"
pro_roslaunch = subprocess.Popen(cmd_roslaunch, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

while True:
    pass







