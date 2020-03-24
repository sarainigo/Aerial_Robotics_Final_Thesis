#!/usr/bin/env python

import rospy
import os
import subprocess
import time
import signal

#se cambia el numero de drones que se envian, la mac de cada uno de los drones y si trabajamos en drone real o simulacion

n_drones = 2
lanzo_gazebo = 1
drone_mac1 = 'C0:1A:71:0E:7D:DA'
drone_mac2 = 'C0:1A:71:13:7D:DA'


# no puedo lanzar desde aqui gazebo xq no puedo lanzar varios a la vez

for i in range(1, n_drones+1):

        if i==1:
            drone_mac = drone_mac1
        if i==2:
            drone_mac = drone_mac2

        print('Lanzando controlador del drone No '+str(i))

        cmd_roslaunch = "roslaunch control_parrot multidrone_control_parrot_launcher.launch arg_num:=" + str(i) + " arg_gazebo:=" + str(lanzo_gazebo) + " arg_mac:=" + drone_mac
        pro_roslaunch = subprocess.Popen(cmd_roslaunch, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)


while True:
    pass



