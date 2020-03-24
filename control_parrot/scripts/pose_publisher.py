#!/usr/bin/env python

import rospy
import time
import threading
import numpy as np
import math


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import AccelStamped
from std_msgs.msg import Header

mutex = threading.Lock()

rospy.init_node('pose_publisher')

# CAMBIAR ESTA VARIABLE EN FUNCION DE SI QUIERO RECIBIR INFO DE OPTITRACK O GAZEBO
gazebo_info = rospy.get_param('~param_gazebo_info', "1")
gazebo_info = int(gazebo_info)

num_drone = rospy.get_param('~param_num', "0")
num_drone = int(num_drone)

print("Gazebo info: " + str(gazebo_info))
print("Num drone: " + str(num_drone))


# publishers
pub_pose = rospy.Publisher('/drone'+str(num_drone)+'/position', PoseStamped, queue_size=10)
pub_twist = rospy.Publisher('/drone'+str(num_drone)+'/twist', TwistStamped, queue_size=10)
pub_accel = rospy.Publisher('/drone'+str(num_drone)+'/accel', AccelStamped, queue_size=10)


def gz_callback(data):
    global drone_pose, flag_rec_info, drone_yaw
    if flag_rec_info == 0:
        flag_rec_info = 1
    mutex.acquire()
    drone_pose.pose.position.x = data.pose.position.x
    drone_pose.pose.position.y = data.pose.position.y
    drone_pose.pose.position.z = data.pose.position.z
    drone_pose.header.stamp = data.header.stamp
    q1 = data.pose.orientation.x
    q2 = data.pose.orientation.y
    q3 = data.pose.orientation.z
    q0 = data.pose.orientation.w
    # Conversion quaternion to euler. Only yaw!=0.
    siny = +2.0 * (q0 * q3 + q1 * q2)
    cosy = +1.0 - 2.0 * (q2 * q2 + q3 * q3)
    drone_yaw = math.atan2(siny, cosy)
    mutex.release()


def ot_callback(data):
    global drone_pose, flag_rec_info, drone_yaw, t_init
    if flag_rec_info == 0:
        flag_rec_info = 1
        t_init.stamp = data.header.stamp

    mutex.acquire()
    drone_pose.pose.position.x = data.pose.position.x
    drone_pose.pose.position.y = data.pose.position.y
    drone_pose.pose.position.z = data.pose.position.z
    drone_pose.header.stamp = data.header.stamp - t_init.stamp

    q1 = data.pose.orientation.x
    q2 = data.pose.orientation.y
    q3 = data.pose.orientation.z
    q0 = data.pose.orientation.w
    # Conversion quaternion to euler. Only yaw!=0.
    siny = +2.0 * (q0 * q3 + q1 * q2)
    cosy = +1.0 - 2.0 * (q2 * q2 + q3 * q3)
    drone_yaw = math.atan2(siny, cosy)
    mutex.release()


def publisher():
    global drone_pose, flag_rec_info, drone_yaw

    act_pose = PoseStamped()
    act_vel = TwistStamped()
    act_accel = AccelStamped()
    drone_yaw = 0
    flag_rec_info = 0

    # subscribers
    if gazebo_info == 1:
        # subscribers gazebo
        print ("Listening to Gazebo \n")
        rospy.Subscriber("/drone_gazebo" + str(num_drone) + "/pose", PoseStamped, gz_callback)
    else:
        print ("Listening to Optitrack \n")
        # subscribers optitrack
        # cambiar nombre de topics
        rospy.Subscriber('/drone_optitrack' + str(num_drone) + '/pose', PoseStamped, ot_callback)

    # Vel minimos cuadrados
    j = 0
    n = 30
    t = np.zeros(n)
    x = np.zeros(n)
    y = np.zeros(n)
    z = np.zeros(n)
    yaw = np.zeros(n)

    # Accel minimos cuadrados
    vt = np.zeros(n)
    vx = np.zeros(n)
    vy = np.zeros(n)
    vz = np.zeros(n)


    rate = rospy.Rate(60)  # 60hz
    while not rospy.is_shutdown():
        if flag_rec_info == 1:
            mutex.acquire()
            act_pose.pose.position.x = drone_pose.pose.position.x
            act_pose.pose.position.y = drone_pose.pose.position.y
            act_pose.pose.position.z = drone_pose.pose.position.z
            act_pose.header.stamp = drone_pose.header.stamp
            act_pose.pose.orientation.z = drone_yaw
            mutex.release()

            # estimacion vel por minimos cuadrados
            for i in range(0, n - 1):
                x[i] = x[i + 1]
                y[i] = y[i + 1]
                z[i] = z[i + 1]
                t[i] = t[i + 1]
                yaw[i] = yaw[i + 1]
            x[n - 1] = float(act_pose.pose.position.x)
            y[n - 1] = float(act_pose.pose.position.y)
            z[n - 1] = float(act_pose.pose.position.z)
            yaw[n - 1] = float(act_pose.pose.orientation.z)
            t[n - 1] = float(act_pose.header.stamp.to_sec())

            # igual es prescindible
            # j contador para no empezar ajuste por minimos cuadrados hasta que no esten n primeras medidas copiadas
            j = j + 1
            if j >= n:
                j = n
                # print("New measure")
                # print("t = ["+str(t[0])+", "+str(t[1])+", "+str(t[2])+", "+str(t[3])+", "+str(t[4])+", "+str(t[5])+", "+str(t[6])+", "+str(t[7])+", "+str(t[8])+", "+str(t[9])+", "+str(t[10])+", "+str(t[11])+", "+str(t[12])+", "+str(t[13])+", "+str(t[14])+", "+str(t[15])+", "+str(t[16])+", "+str(t[17])+", "+str(t[18])+", "+str(t[19])+"]")
                # print("x = [" + str(x[0]) + ", " + str(x[1]) + ", " + str(x[2]) + ", " + str(x[3]) + ", " + str(x[4]) + ", " + str(x[5]) + ", " + str(x[6]) + ", " + str(x[7]) + ", " + str(x[8]) + ", " + str(x[9]) + ", " + str(x[10]) + ", " + str(x[11]) + ", " + str(x[12]) + ", " + str(x[13]) + ", " + str(x[14]) + ", " + str(x[15]) + ", " + str(x[16]) + ", " + str(x[17]) + ", " + str(x[18]) + ", " + str(x[19]) + "]")

                # Coefs pol posicion
                cx = np.polyfit(t, x, 2)
                cy = np.polyfit(t, y, 2)
                cz = np.polyfit(t, z, 2)
                cyaw = np.polyfit(t, yaw, 2)

                # pols
                polvx = np.polyder(np.poly1d(cx))
                polvy = np.polyder(np.poly1d(cy))
                polvz = np.polyder(np.poly1d(cz))
                polvyaw = np.polyder(np.poly1d(cyaw))

                act_vel.twist.linear.x = polvx(t[n-1])
                act_vel.twist.linear.y = polvy(t[n-1])
                act_vel.twist.linear.z = polvz(t[n-1])
                act_vel.twist.angular.z = polvyaw(t[n-1])
                act_vel.header.stamp = act_pose.header.stamp

                polax = np.polyder(polvx)
                polay = np.polyder(polvy)
                polaz = np.polyder(polvz)
                act_accel.accel.linear.x = polax(t[n-1])
                act_accel.accel.linear.y = polay(t[n-1])
                act_accel.accel.linear.z = polaz(t[n-1])
                act_accel.header.stamp = act_pose.header.stamp


                # print ("vx: " + str(act_vel.twist.linear.x) + " vy: " + str(act_vel.twist.linear.y) + "vz: " + str(act_vel.twist.linear.z))
                pub_pose.publish(act_pose)
                pub_twist.publish(act_vel)
                pub_accel.publish(act_accel)
                print('New measure')
                print ('time = ' + str(act_pose.header.stamp.to_sec()) + 'secs')
                print ('x = ' + str(act_pose.pose.position.x)+', y = ' + str(act_pose.pose.position.y)+', z = '+ str(act_pose.pose.position.z))
                print ('yaw = ' + str(act_pose.pose.orientation.z))
                print ('vx = ' + str(act_vel.twist.linear.x) + ', vy = ' + str(act_vel.twist.linear.y) + ', vz = ' + str(act_vel.twist.linear.z))
                print ('vyaw = ' + str(act_vel.twist.angular.z))
                print ('ax = ' + str(act_accel.accel.linear.x) + ', ay = ' + str(act_accel.accel.linear.y) + ', az = ' + str(act_accel.accel.linear.z))


                if x[n-1] > 0.1 and x[n-1] < 0.4:
                    np.savetxt('plot_mmc_x1.txt', x)
                    np.savetxt('plot_mmc_yaw1.txt', yaw)
                    np.savetxt('plot_mmc_t1.txt', t)
                    np.savetxt('plot_mmc_vx1.txt', vx)
                    np.savetxt('plot_mmc_vt1.txt', vt)
                    plot_calculo_vels1 = [act_vel.twist.linear.x, act_vel.twist.angular.z]
                    np.savetxt('plot_calculo_vels1.txt', plot_calculo_vels1)
                if x[n-1] > 0.5 and x[n-1] < 0.7:
                    np.savetxt('plot_mmc_x2.txt', x)
                    np.savetxt('plot_mmc_yaw2.txt', yaw)
                    np.savetxt('plot_mmc_t2.txt', t)
                    np.savetxt('plot_mmc_vx2.txt', vx)
                    np.savetxt('plot_mmc_vt2.txt', vt)
                    plot_calculo_vels2 = [act_vel.twist.linear.x, act_vel.twist.angular.z]
                    np.savetxt('plot_calculo_vels2.txt', plot_calculo_vels2)
                if x[n-1] > 0.8 and x[n-1] < 1:
                    np.savetxt('plot_mmc_x3.txt', x)
                    np.savetxt('plot_mmc_yaw3.txt', yaw)
                    np.savetxt('plot_mmc_t3.txt', t)
                    np.savetxt('plot_mmc_vx3.txt', vx)
                    np.savetxt('plot_mmc_vt3.txt', vt)
                    plot_calculo_vels3 = [act_vel.twist.linear.x, act_vel.twist.angular.z]
                    np.savetxt('plot_calculo_vels3.txt', plot_calculo_vels3)
        rate.sleep()


if __name__ == '__main__':

    global drone_pose, flag_rec_info, t_init
    drone_pose = PoseStamped()
    flag_rec_info = 0
    t_init = Header()

    try:
        publisher()
    except rospy.ROSInterruptException:
        pass


