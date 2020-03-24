#!/usr/bin/env python

import rospy
import os
import subprocess
import time
import signal

# ya hay hecho hasta a0=0.1 a1=0.2 a2=0.3

my_a0 = ['0.0', '0.2', '0.4', '0.6', '0.8', '1.0']
my_a1 = ['-1.2', '-1.0', '-0.8', '-0.6', '-0.4', '-0.2', '0.0', '0.2', '0.4', '0.6', '0.8', '1.0', '1.2']
my_a2 = ['-1.2', '-1.0', '-0.8', '-0.6', '-0.4', '-0.2', '0.0', '0.2', '0.4', '0.6', '0.8', '1.0', '1.2']


my_w1_x = 0
my_w1_y = -1
my_w2_x = -1
my_w2_y = 0

my_a0_len = len(my_a0)
my_a1_len = len(my_a1)
my_a2_len = len(my_a2)

n_actual = 1
n_total = my_a0_len * my_a1_len * my_a2_len

for k in range(0, my_a2_len):
    for j in range(0, my_a1_len):
        for i in range(0, my_a0_len):

            while True:
                a = 0
                print("Ejecucion "+str(n_actual)+' de '+str(n_total) + ' comenzando')

                print("LAUNCH SPHINX")
                cmd_sphinx = "sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/airborne.drone"
                pro_sphinx = subprocess.Popen(cmd_sphinx, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                time.sleep(10)
                cmd_roslaunch = "roslaunch control_parrot simulations_carrotchasing_control_parrot_launcher.launch arg_a0:=" + my_a0[i] + " arg_a1:=" + my_a1[j] + " arg_a2:=" + my_a2[k] + " arg_w1_x:=" + str(my_w1_x) + " arg_w1_y:=" + str(my_w1_y) + " arg_w2_x:=" + str(my_w2_x) + " arg_w2_y:=" + str(my_w2_y)
                pro_roslaunch = subprocess.Popen(cmd_roslaunch, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

                print('a0='+my_a0[i]+'a1='+my_a1[j]+'a2='+my_a2[k])

                while a<50:
                    print ("Drone moviendose. Segundo "+str(a)+"/50")
                    time.sleep(1)
                    a = a + 1

                print("KILL CONTROL_PARROT.LAUNCH")
                os.killpg(os.getpgid(pro_roslaunch.pid), signal.SIGTERM)
                os.killpg(os.getpgid(pro_sphinx.pid), signal.SIGTERM)
                os.system("rosnode kill -a")
                os.system("killall -9 rosmaster")
                os.system("killall -9 gazebo_info")
                a0 = float(my_a0[i])
                a1 = float(my_a1[j])
                a2 = float(my_a2[k])
                name = "./data/info_drone_" + str(a0) + "_" + str(a1) + "_" + str(a2) + ".txt"

                if os.path.isfile(name):
                    break

                else:
                    print("Error en la simulacion")

            print("Ejecucion "+str(n_actual)+' de '+str(n_total)+' terminada')
            n_actual = n_actual+1

            time.sleep(10)




