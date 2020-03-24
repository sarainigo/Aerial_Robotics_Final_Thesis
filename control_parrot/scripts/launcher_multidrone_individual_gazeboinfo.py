#!/usr/bin/env python

import rospy
import os
import subprocess
import time
import signal

# se lanza este nodo en cada uno de los ordenadores donde se lancen los simuladores, cambiando el valor de "drone"

drone = 2
# no puedo lanzar desde aqui gazebo xq no puedo lanzar varios a la vez

print("LANZANDO SPHINX DE DRONE "+str(drone))
cmd_sphinx = "sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/airborne.drone"
pro_sphinx = subprocess.Popen(cmd_sphinx, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
time.sleep(10)
print('LANZANDO GAZEBO INFO DE DRONE '+str(drone))
cmd_roslaunch = "roslaunch control_parrot multidrone_control_parrot_launcher_individual_gazeboinfo.launch arg_num:=" + str(drone)
pro_roslaunch = subprocess.Popen(cmd_roslaunch, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)


while True:
    pass



