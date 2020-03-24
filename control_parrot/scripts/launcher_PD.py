#!/usr/bin/env python

import rospy
import os
import subprocess
import time
import signal

# 17*16*7*120(seg)= 1904*120=228480seg=3808min=63,47horas
# 16*16*6 = 51,2h = miercoles a las 7-8 de la tarde

# my_w = ['0.15', '0.20', '0.25', '0.30', '0.35', '0.4', '0.50', '0.60','0.70','0.80', '0.9', '1', '1.5', '2', '2.5', '3']

# como primera aproximacion se considerara que el comportamiento en x e y es igual
# my_kp = ['0', '0.1', '0.2',  '0.3', '0.4', '0.5', '0.6', '0.7', '0.8', '0.9', '1', '1.1', '1.2', '1.3', '1.4','1.5']
# my_kd = ['0', '0.1', '0.2', '0.3', '0.4', '0.5']


my_w = ['0.15', '0.20', '0.25', '0.30', '0.35', '0.4', '0.50', '0.60','0.70','0.80', '0.9', '1', '1.5', '2', '2.5', '3']

# como primera aproximacion se considerara que el comportamiento en x e y es igual
my_kp = ['0', '0.1', '0.2',  '0.3', '0.4', '0.5', '0.6', '0.7', '0.8', '0.9', '1', '1.1', '1.2', '1.3', '1.4','1.5']
my_kd = ['0.3', '0.4', '0.5']

#cuidado kd=0.3 y kp=[0,0.8] ya hecho

my_w_len = len(my_w)
my_kp_len = len(my_kp)
my_kd_len = len(my_kd)

n_actual = 1
n_total = my_w_len * my_kp_len * my_kd_len

for k in range(0, my_kd_len):
    for j in range(0, my_kp_len):
        for i in range(0, my_w_len):

            while True:
                a = 0
                print("Ejecucion "+str(n_actual)+' de '+str(n_total) + ' comenzando')

                print("LAUNCH SPHINX")
                cmd_sphinx = "sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/airborne.drone"
                pro_sphinx = subprocess.Popen(cmd_sphinx, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                time.sleep(10)
                cmd_roslaunch = "roslaunch control_parrot simulations_PD_control_parrot_launcher.launch arg_vel:=" + my_w[i] + " arg_kp:=" + my_kp[j] + " arg_kd:=" + my_kd[k]
                pro_roslaunch = subprocess.Popen(cmd_roslaunch, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

                print("LAUNCH "+ str(i) +" OF CONTROL_PARROT.LAUNCH")
                print('kd='+my_kd[k]+'kp='+my_kp[j]+'w='+my_w[i])

                while a<100:
                    print ("Drone moviendose. Segundo "+str(a)+"/100")
                    time.sleep(1)
                    a = a + 1

                print("KILL CONTROL_PARROT.LAUNCH")
                os.killpg(os.getpgid(pro_roslaunch.pid), signal.SIGTERM)
                os.killpg(os.getpgid(pro_sphinx.pid), signal.SIGTERM)
                os.system("rosnode kill -a")
                os.system("killall -9 rosmaster")
                os.system("killall -9 gazebo_info")
                w = float(my_w[i])
                kp = float(my_kp[j])
                kd = float(my_kd[k])
                name = "./data/info_drone_" + str(w) + "_" + str(kp) + "_" + str(kd) + ".txt"

                if os.path.isfile(name):
                    break

                else:
                    print("Error en la simulacion")

            print("Ejecucion "+str(n_actual)+' de '+str(n_total)+' terminada')
            n_actual = n_actual+1

            time.sleep(10)




