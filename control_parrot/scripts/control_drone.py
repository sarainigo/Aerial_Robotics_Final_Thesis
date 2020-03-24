#!/usr/bin/env python
import rospy
import time
import threading
import minidrone
import dronedict
import math
import numpy as np
import roslib
roslib.load_manifest('control_parrot')
import actionlib
import control_parrot.msg

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from control_parrot.srv import *

# Banderas estado conexion
S_DISCONNECTED = 0
S_CONNECTING = 1
S_CONNECTED = 2

# Banderas estado vuelo
D_GROUND = 0
D_ACTION_ON = 1
D_FLYINGFREE = 2
D_ACTION_STAYIN = 3
D_EMERGENCY = 4
D_CHANGEBAT = 5
D_VELOCITY_CMD = 6
D_ACTION_CMD = 7

#simulador
# DRONEMAC = 'C0:1A:71:0E:7D:DA'
#simulador usb malo
# DRONEMAC = C0:1A:71:13:7D:DA
# realidad drone 1
# DRONEMAC = 'E0:14:F5:F4:3D:C3'
CB_MSG = 0
CB_BATTERY = 1
CB_DATA_UPDATE = 2
CB_SPEED = 3
CB_STATE = 4

rospy.init_node('control_drone')

num_drone = rospy.get_param('~param_num', 0)
num_drone = int(num_drone)

DRONEMAC = rospy.get_param('~param_mac', 'C0:1A:71:0E:7D:DA')
DRONEMAC = str(DRONEMAC)

mutex = threading.Lock()


# FUNCION ACTUALIZACION VARIABLES DRONE REAL


def refresh_data(t, data):
    global message, config, battery, speed, state
    if t == CB_MSG:
        mutex.acquire()
        message = data
        mutex.release()
    elif t == CB_BATTERY:
        mutex.acquire()
        battery = data
        mutex.release()
    elif t == CB_SPEED:
        mutex.acquire()
        speed = data
        mutex.release()
    elif t == CB_DATA_UPDATE:
        mutex.acquire()
        config = data
        mutex.release()
    elif t == CB_STATE:
        mutex.acquire()
        state = S_CONNECTED if data == 'y' else S_DISCONNECTED
        mutex.release()


# DEFINICION DE CALLBACKS DE TOPICS


def pose_callback(msg):
    global drone_pose, drone_yaw
    mutex.acquire()
    drone_pose.pose.position.x = msg.pose.position.x
    drone_pose.pose.position.y = msg.pose.position.y
    drone_pose.pose.position.z = msg.pose.position.z
    drone_pose.header.stamp = msg.header.stamp
    drone_yaw = msg.pose.orientation.z
    mutex.release()


def twist_callback(msg):
    global drone_twist
    mutex.acquire()
    drone_twist.twist.linear.x = msg.twist.linear.x
    drone_twist.twist.linear.y = msg.twist.linear.y
    drone_twist.twist.linear.z = msg.twist.linear.z
    drone_twist.twist.angular.z = msg.twist.angular.z
    drone_twist.header.stamp = msg.header.stamp
    mutex.release()


def accel_callback(msg):
    global drone_accel
    mutex.acquire()
    drone_accel.accel.linear.x = msg.accel.linear.x
    drone_accel.accel.linear.y = msg.accel.linear.y
    drone_accel.accel.linear.z = msg.accel.linear.z
    drone_accel.header.stamp = msg.header.stamp
    mutex.release()


def cmd_incr_vel_callback(msg):
    global drone_yaw
    mutex.acquire()
    yaw = drone_yaw
    mutex.release()
    vx = math.cos(yaw) * msg.linear.x - math.sin(yaw) * msg.linear.y
    vy = math.sin(yaw) * msg.linear.x + math.cos(yaw) * msg.linear.y
    vz = msg.linear.z
    t_z = msg.angular.z
    drone.move(vx, vy, vz, t_z)
    if msg.angular.z == 0:
        rospy.loginfo('Moving drone')
    else:
        rospy.loginfo('Spinning drone')


def cmd_velocity_callback(msg):
    # no implementado el comando vz y vyaw en m/s, asique les meto un 30porciento siempre, independientemente de los m/s que me digan
    global drone_pose, drone_yaw, drone_twist, state, drone_accel
    global flag_action, client, last_position
    mutex.acquire()
    flag = flag_action
    mutex.release()
    if flag == D_ACTION_STAYIN:
        client.cancel_goal()
    mutex.acquire()
    flag_action = D_VELOCITY_CMD
    mutex.release()

    phi_des = np.arctan2(msg.linear.y, msg.linear.x)
    Va_des = np.linalg.norm([msg.linear.x, msg.linear.y])

    if msg.linear.z == 0.0 or msg.linear.z == 0:
        vz = 0
    else:
        vz = 30

    if msg.angular.z == 0.0 or msg.linear.z == 0:
        vyaw = 0
    else:
        vyaw = 30

    kp_vax = 0.7
    kp_vay = 0.7
    kd_vax = 0.3
    kd_vay = 0.3

    mutex.acquire()
    s = state
    act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
    act_t = drone_pose.header.stamp.to_sec()
    act_yaw = drone_yaw
    act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y, drone_twist.twist.linear.z,
               drone_twist.twist.angular.z]
    act_accel = [drone_accel.accel.linear.x, drone_accel.accel.linear.y, drone_accel.accel.linear.z]
    mutex.release()

    # CONTROL EN VELOCIDAD (necesito phi_des y Va_des en m/s)
    # proyeccion en x e y de phi_des
    phi_des_x = math.cos(phi_des)
    phi_des_y = math.sin(phi_des)

    Vax_des = Va_des * phi_des_x
    Vay_des = Va_des * phi_des_y

    pdx_va = (Vax_des - act_vel[0]) * kp_vax + act_accel[0] * kd_vax
    pdy_va = (Vay_des - act_vel[1]) * kp_vay + act_accel[1] * kd_vay

    # anadimos a cmd_vel el pd
    Vax_des_pd = Vax_des + pdx_va
    Vay_des_pd = Vay_des + pdy_va

    # pasamos de m/s a porcentaje
    coefx = [-0.000668940781647, -0.0326762435076, -0.0294549051165, 16.1544748879, 0.162490399212]
    coefy = [0.00687112729217, -0.014779740721, -0.11999831192, 18.1045243274, -0.0331072136715]
    polx = np.poly1d(coefx)
    poly = np.poly1d(coefy)
    Vax_des_porcent = polx(Vax_des_pd)
    Vay_des_porcent = poly(Vay_des_pd)

    act_phi = np.arctan2(act_vel[1], act_vel[0])
    act_Va = np.linalg.norm([act_vel[0], act_vel[1]])
    act_phi_x = math.cos(act_phi)
    act_phi_y = math.sin(act_phi)

    # comando de movimiento con control pd
    cmd_x = Vax_des_porcent
    cmd_y = Vay_des_porcent
    cmd_va = np.linalg.norm([cmd_x, cmd_y])

    print ("cmd_x: " + str(cmd_x) + " cmd_y: " + str(cmd_y))
    vx = math.cos(act_yaw) * cmd_x + math.sin(act_yaw) * cmd_y
    vy = math.sin(act_yaw) * cmd_x - math.cos(act_yaw) * cmd_y
    np.clip(vx, -80, 80)
    np.clip(vy, -80, 80)
    print ("vx_cmd: " + str(vx) + " vy_cmd: " + str(vy))
    cmd_va_drone = np.linalg.norm([vx, vy])

    drone.move(vx, vy, vz, vyaw)

    # SALVAR EN TXT VARIABLES
    vector_info_global = [act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t]
    f = open("./data/info_vuelo.txt", "a+")
    for data in vector_info_global:
        f.write(str(data) + " ")
    f.write("\n")
    f.close()

    rospy.loginfo('Moving drone')


# DEFINICION DE CALLBACKS DE SERVICIOS

def handle_change_mode(req):
    rospy.loginfo('Changing operation mode')
    global flag_control_mode
    global flag_action
    flag_control_mode = req.md
    if flag_action == D_ACTION_STAYIN:
        client.cancel_goal()
    if req.md==1:
        mutex.acquire()
        flag_action = D_VELOCITY_CMD
        mutex.release()
    if req.md==0:
        mutex.acquire()
        flag_action = D_ACTION_CMD
        mutex.release()
    return ChangeModeServiceResponse("Operation mode changed to "+str(req.md))


def handle_disconnect(req):
    rospy.loginfo('Disconnect instruction (function not implemented yet to be able to reconnect)')
    drone.disconnect()
    return DroneServiceResponse("Drone disconnected")


def handle_increase(req):
    rospy.loginfo('Incrementing speed of the drone')
    drone.incr_speed()
    return DroneServiceResponse("Increase of speed done. v=" + str(drone.speed))


def handle_decrease(req):
    rospy.loginfo('Decrementing speed of the drone')
    drone.decr_speed()
    return DroneServiceResponse("Decrease of speed done. v=" + str(drone.speed))


def handle_emergency(req):
    rospy.loginfo('Emergency instruction')
    drone.emergency()
    return DroneServiceResponse("Emergency done")


# DEFINICION DE SERVERS DE ACCIONES


class ConnectServer(object):
    # create messages that are used to publish feedback/result
    _feedback = control_parrot.msg.ConnectFeedback()
    _result = control_parrot.msg.ConnectResult()
    def __init__(self, name):
        self.server_name = name
        self.server = actionlib.SimpleActionServer(self.server_name, control_parrot.msg.ConnectAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        global state
        success = True
        mutex.acquire()
        state = S_CONNECTING
        mutex.release()
        rospy.loginfo('Connecting to drone')
        drone.connect()
        mutex.acquire()
        self._feedback.state = state
        mutex.release()
        # cuidado, doy por hecho que se puede salir de la funcion drone.connect() con state = S_CONNECTING o S_DISCONNECTED.
        # No he visto ningun caso en el que esto ocurra, por lo que el feedback y el while podrian ser prescindibles
        while self._feedback.state != S_CONNECTED:
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.server_name)
                self.server.set_preempted()
                success = False
                break
            mutex.acquire()
            self._feedback.state = state
            mutex.release()
            # publish the feedback
            self.server.publish_feedback(self._feedback)
        if success:
            self._result.result = self._feedback.state
            print ('Connection succeeded')
            self.server.set_succeeded(self._result)


class LandServer(object):
    # create messages that are used to publish feedback/result
    _feedback = control_parrot.msg.LandFeedback()
    _result = control_parrot.msg.LandResult()
    def __init__(self, name):
        self.server_name = name
        self.server = actionlib.SimpleActionServer(self.server_name, control_parrot.msg.LandAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        global drone_pose, state
        global flag_action, client, last_position
        global cont_accion_land
        cont_accion_land = cont_accion_land + 1

        mutex.acquire()
        flag = flag_action
        mutex.release()
        if flag == D_ACTION_STAYIN:
            client.cancel_goal()
        mutex.acquire()
        flag_action = D_ACTION_ON
        mutex.release()
        success = True
        disconnect = False
        mutex.acquire()
        self._feedback.z = drone_pose.pose.position.z
        s = state
        mutex.release()
        rospy.loginfo('Landing the drone')
        drone.land()
        while self._feedback.z > 0.04:
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.server_name)
                self.server.set_preempted()
                success = False
                break
            if s != S_CONNECTED:
                rospy.loginfo('Drone disconnected')
                disconnect = True
                break
            mutex.acquire()
            self._feedback.z = drone_pose.pose.position.z
            mutex.release()
            # publish the feedback
            self.server.publish_feedback(self._feedback)

            # SALVAR EN TXT VARIABLES
            mutex.acquire()
            act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z, drone_yaw]
            act_t = drone_pose.header.stamp.to_sec()
            mutex.release()
            vector_info = [act_pose[0], act_pose[1], act_pose[2], act_pose[3]]
            '''f = open("./data/info_drone_land"+str(cont_accion_land)+".txt", "a+")
            for data in vector_info:
                f.write(str(data) + " ")
            f.write("\n")
            f.close()
            vector_info_global = [act_pose[0], act_pose[1], act_pose[2], act_pose[3], act_t]
            f = open("./data/info_vuelo.txt", "a+")
            for data in vector_info_global:
                f.write(str(data) + " ")
            f.write("\n")
            f.close()'''


        if disconnect:
            self._result.final_z = self._feedback.z
            print ('Exit of action')
            self.server.set_succeeded(self._result)
        elif success:
            self._result.final_z = self._feedback.z
            print ('Land succeeded')
            self.server.set_succeeded(self._result)
        mutex.acquire()
        last_position = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z, drone_yaw]
        flag_action = D_GROUND
        mutex.release()


class TakeOffServer(object):
    # create messages that are used to publish feedback/result
    _feedback = control_parrot.msg.TakeOffFeedback()
    _result = control_parrot.msg.TakeOffResult()
    def __init__(self, name):
        self.server_name = name
        self.server = actionlib.SimpleActionServer(self.server_name, control_parrot.msg.TakeOffAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        global drone_pose, state, drone_yaw
        global flag_action, client, last_position
        global cont_accion_takeoff
        cont_accion_takeoff = cont_accion_takeoff + 1

        mutex.acquire()
        flag = flag_action
        mutex.release()
        if flag == D_ACTION_STAYIN:
            client.cancel_goal()
        mutex.acquire()
        flag_action = D_ACTION_ON
        mutex.release()
        success = True
        disconnect = False
        z_takeoff = 0.75
        mutex.acquire()
        self._feedback.z = drone_pose.pose.position.z
        s = state
        pose_init = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_yaw]
        mutex.release()
        rospy.loginfo('Taking off the drone')
        drone.takeoff()
        while self._feedback.z < z_takeoff:
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.server_name)
                self.server.set_preempted()
                success = False
                break
            if s != S_CONNECTED:
                rospy.loginfo('Drone disconnected')
                disconnect = True
                break
            mutex.acquire()
            self._feedback.z = drone_pose.pose.position.z
            mutex.release()
            # publish the feedback
            self.server.publish_feedback(self._feedback)

            # SALVAR EN TXT VARIABLES
            mutex.acquire()
            act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z, drone_yaw]
            act_t = drone_pose.header.stamp.to_sec()
            mutex.release()
            vector_info = [act_pose[0], act_pose[1], act_pose[2], act_pose[3]]
            '''f = open("./data/info_drone_takeoff"+str(cont_accion_takeoff)+".txt", "a+")
            for data in vector_info:
                f.write(str(data) + " ")
            f.write("\n")
            f.close()
            vector_info_global = [act_pose[0], act_pose[1], act_pose[2], act_pose[3], act_t]
            f = open("./data/info_vuelo.txt", "a+")
            for data in vector_info_global:
                f.write(str(data) + " ")
            f.write("\n")
            f.close()'''


        if disconnect:
            self._result.final_z = self._feedback.z
            print ('Exit of action')
            self.server.set_succeeded(self._result)
        elif success:
            self._result.final_z = self._feedback.z
            print ('Take off succeeded')
            self.server.set_succeeded(self._result)
        mutex.acquire()
        last_position = [pose_init[0], pose_init[1], drone_pose.pose.position.z, pose_init[2]]
        flag_action = D_FLYINGFREE
        mutex.release()


class GotoWaypointServer(object):
    # create messages that are used to publish feedback/result
    _feedback = control_parrot.msg.GotoWaypointFeedback()
    _result = control_parrot.msg.GotoWaypointResult()
    def __init__(self, name):
        self.server_name = name
        self.server = actionlib.SimpleActionServer(self.server_name, control_parrot.msg.GotoWaypointAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        global drone_pose, drone_yaw, drone_twist, state, drone_accel
        global flag_action, client, last_position
        global cont_accion_goto
        cont_accion_goto = cont_accion_goto + 1

        mutex.acquire()
        flag = flag_action
        mutex.release()
        if flag == D_ACTION_STAYIN:
            client.cancel_goal()
        mutex.acquire()
        flag_action = D_ACTION_ON
        mutex.release()
        success = True
        disconnect = False
        # Control PD
        # kpx = 100
        # kdx = 50
        # kpy = 100
        # kdy = 50
        # kpz = 100
        # kdz = 0.0
        # kpyaw = 90
        # kdyaw = 80

        kpx = 100
        kdx = 70
        kpy = 100
        kdy = 70
        kpz = 70
        kdz = 0.0
        kpyaw = 30
        kdyaw = 25

        thr = 0.08
        yawthr = 0.1
        count_stable = 45
        # 45/15hz = 3 segs reales

        # inicializacion
        des_pose = [goal.des_pose.position.x, goal.des_pose.position.y, goal.des_pose.position.z]
        des_yaw = goal.des_pose.orientation.z
        mutex.acquire()
        s = state
        act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
        act_t = drone_pose.header.stamp.to_sec()
        act_yaw = drone_yaw
        act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y,  drone_twist.twist.linear.z, drone_twist.twist.angular.z]
        self._feedback.act_pose.position.x = act_pose[0]
        self._feedback.act_pose.position.y = act_pose[1]
        self._feedback.act_pose.position.z = act_pose[2]
        self._feedback.act_pose.orientation.z = act_yaw
        mutex.release()

        rospy.loginfo('Moving drone to waypoint [' + str(goal.des_pose.position.x) +', ' + str(goal.des_pose.position.y) +', ' + str(goal.des_pose.position.z)+']')
        rate = rospy.Rate(15)  # 15hz
        count = 0
        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.server_name)
                self.server.set_preempted()
                success = False
                break
            if s != S_CONNECTED:
                rospy.loginfo('Drone disconnected')
                disconnect = True
                break
            # refresh values
            mutex.acquire()
            act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
            act_t = drone_pose.header.stamp.to_sec()
            act_yaw = drone_yaw
            act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y, drone_twist.twist.linear.z, drone_twist.twist.angular.z]
            mutex.release()
            distance = math.sqrt(math.pow(act_pose[0] - des_pose[0], 2) + math.pow(act_pose[1] - des_pose[1], 2) + math.pow(act_pose[2] - des_pose[2], 2))
            yawerror = math.fabs(act_yaw-des_yaw)
            self._feedback.act_pose.position.x = act_pose[0]
            self._feedback.act_pose.position.y = act_pose[1]
            self._feedback.act_pose.position.z = act_pose[2]
            self._feedback.act_pose.orientation.z = act_yaw
            # publish the feedback
            self.server.publish_feedback(self._feedback)
            if distance < thr and yawerror < yawthr:
                count = count + 1
            if count == count_stable:
                break
            # PD
            pd_x = (des_pose[0] - act_pose[0]) * kpx - act_vel[0] * kdx
            pd_y = (des_pose[1] - act_pose[1]) * kpy - act_vel[1] * kdy
            pd_z = (des_pose[2] - act_pose[2]) * kpz - act_vel[2] * kdz
            p_yaw = (des_yaw - act_yaw) * kpyaw - act_vel[3] * kdyaw
            print ('New measure goto_waypoint')
            print ("pd_x: " + str(pd_x) + " pd_y: " + str(pd_y) + " pd_z: " + str(pd_z) + " pd_yaw: " + str(p_yaw))
            vx = math.cos(act_yaw) * pd_x + math.sin(act_yaw) * pd_y
            vy = math.sin(act_yaw) * pd_x - math.cos(act_yaw) * pd_y
            vz = pd_z
            vyaw = -p_yaw

            np.clip(vx, -50, 50)
            np.clip(vy, -50, 50)
            np.clip(vyaw, -50, 50)
            print ("vx_cmd: " + str(vx) + " vy_cmd: " + str(vy) + " vz_cmd: " + str(vz) + " vyaw_cmd: " + str(vyaw))
            distance = math.sqrt(
                math.pow(act_pose[0] - des_pose[0], 2) + math.pow(act_pose[1] - des_pose[1], 2) + math.pow(
                    act_pose[2] - des_pose[2], 2))
            print ("distance: " + str(distance))
            yawerror = math.fabs(act_yaw - des_yaw)
            print ("yaw error: " + str(yawerror))
            drone.move(vx, vy, vz, vyaw)

            # SALVAR EN TXT VARIABLES
            error_x = des_pose[0] - act_pose[0]
            error_y = des_pose[1] - act_pose[1]
            error_z = des_pose[2] - act_pose[2]
            error_yaw = des_yaw - act_yaw
            vector_info = [act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t, act_vel[0], act_vel[1], act_vel[2], act_vel[3], pd_x, pd_y, pd_z, p_yaw, distance, error_x, error_y, error_z, error_yaw, des_pose[0], des_pose[1], des_pose[2], kpx, kdx, kpy, kdy, kpz, kdz, des_yaw, kpyaw, kdyaw, thr, yawthr, count_stable]
            '''f = open("./data/info_drone_goto"+str(cont_accion_goto)+".txt", "a+")
            for data in vector_info:
                f.write(str(data) + " ")
            f.write("\n")
            f.close()
            vector_info_global = [act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t]
            f = open("./data/info_vuelo.txt", "a+")
            for data in vector_info_global:
                f.write(str(data) + " ")
            f.write("\n")
            f.close()'''

            rate.sleep()

        if disconnect:
            self._result.final_pose.position.x = self._feedback.act_pose.position.x
            self._result.final_pose.position.y = self._feedback.act_pose.position.y
            self._result.final_pose.position.z = self._feedback.act_pose.position.z
            self._result.final_pose.orientation.z = self._feedback.act_pose.orientation.z
            print ('Exit of action')
            self.server.set_succeeded(self._result)
        elif success:
            self._result.final_pose.position.x = self._feedback.act_pose.position.x
            self._result.final_pose.position.y = self._feedback.act_pose.position.y
            self._result.final_pose.position.z = self._feedback.act_pose.position.z
            self._result.final_pose.orientation.z = self._feedback.act_pose.orientation.z
            print ('Arrival to waypoint succeeded')
            self.server.set_succeeded(self._result)

        mutex.acquire()
        last_position = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z, drone_yaw]
        flag_action = D_FLYINGFREE
        mutex.release()


class StayinWaypointServer(object):
    # create messages that are used to publish feedback/result
    _feedback = control_parrot.msg.GotoWaypointFeedback()
    _result = control_parrot.msg.GotoWaypointResult()
    def __init__(self, name):
        self.server_name = name
        self.server = actionlib.SimpleActionServer(self.server_name, control_parrot.msg.GotoWaypointAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        global drone_pose, drone_yaw, drone_twist, state
        global flag_action
        global cont_accion_stayin
        cont_accion_stayin = cont_accion_stayin + 1

        # prescindible porque ya lo he cambiado
        mutex.acquire()
        flag_action = D_ACTION_STAYIN
        mutex.release()
        success = True
        disconnect = False
        # Control PD
        kpx = 100
        kdx = 50
        kpy = 100
        kdy = 50
        kpz = 100
        kdz = 0.0
        kpyaw = 90
        kdyaw = 80

        # inicializacion
        des_pose = [goal.des_pose.position.x, goal.des_pose.position.y, goal.des_pose.position.z]
        des_yaw = goal.des_pose.orientation.z
        mutex.acquire()
        s = state
        act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
        act_t = drone_pose.header.stamp.to_sec()
        act_yaw = drone_yaw
        act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y, drone_twist.twist.linear.z, drone_twist.twist.angular.z]
        self._feedback.act_pose.position.x = act_pose[0]
        self._feedback.act_pose.position.y = act_pose[1]
        self._feedback.act_pose.position.z = act_pose[2]
        self._feedback.act_pose.orientation.z = act_yaw
        mutex.release()

        rospy.loginfo('Stay in waypoint [' + str(goal.des_pose.position.x) +', ' + str(goal.des_pose.position.y) +', ' + str(goal.des_pose.position.z)+']')

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.server_name)
                self.server.set_preempted()
                success = False
                break
            if s != S_CONNECTED:
                rospy.loginfo('Drone disconnected')
                disconnect = True
                break
            # refresh values
            mutex.acquire()
            act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
            act_t = drone_pose.header.stamp.to_sec()
            act_yaw = drone_yaw
            act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y, drone_twist.twist.linear.z, drone_twist.twist.angular.z]
            mutex.release()
            self._feedback.act_pose.position.x = act_pose[0]
            self._feedback.act_pose.position.y = act_pose[1]
            self._feedback.act_pose.position.z = act_pose[2]
            self._feedback.act_pose.orientation.z = act_yaw
            # publish the feedback
            self.server.publish_feedback(self._feedback)
            # PD
            pd_x = (des_pose[0] - act_pose[0]) * kpx - act_vel[0] * kdx
            pd_y = (des_pose[1] - act_pose[1]) * kpy - act_vel[1] * kdy
            pd_z = (des_pose[2] - act_pose[2]) * kpz - act_vel[2] * kdz
            p_yaw = (des_yaw - act_yaw) * kpyaw - act_vel[3] * kdyaw
            print ('New measure stayin_waypoint')
            print ("pd_x: " + str(pd_x) + " pd_y: " + str(pd_y) + " pd_z: " + str(pd_z) + " pd_yaw: " + str(p_yaw))
            vx = math.cos(act_yaw) * pd_x + math.sin(act_yaw) * pd_y
            vy = math.sin(act_yaw) * pd_x - math.cos(act_yaw) * pd_y
            vz = pd_z
            vyaw = -p_yaw
            np.clip(vx, -80, 80)
            np.clip(vy, -80, 80)
            np.clip(vz, -80, 80)
            np.clip(vyaw, -80, 80)
            print ("vx_cmd: " + str(vx) + " vy_cmd: " + str(vy) + " vz_cmd: " + str(vz) + " vyaw_cmd: " + str(vyaw))
            distance = math.sqrt(math.pow(act_pose[0] - des_pose[0], 2) + math.pow(act_pose[1] - des_pose[1], 2) + math.pow(act_pose[2] - des_pose[2], 2))
            print ("distance: " + str(distance))
            yawerror = math.fabs(act_yaw - des_yaw)
            print ("yaw error: " + str(yawerror))
            drone.move(vx, vy, vz, vyaw)

            # SALVAR EN TXT VARIABLES
            error_x = des_pose[0] - act_pose[0]
            error_y = des_pose[1] - act_pose[1]
            error_z = des_pose[2] - act_pose[2]
            error_yaw = des_yaw - act_yaw
            vector_info = [act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t, act_vel[0], act_vel[1], act_vel[2], act_vel[3], pd_x, pd_y, pd_z, p_yaw, distance, error_x, error_y, error_z, error_yaw, des_pose[0], des_pose[1], des_pose[2], kpx, kdx, kpy, kdy, kpz, kdz, des_yaw, kpyaw, kdyaw]
            '''f = open("./data/info_drone_stayin"+str(cont_accion_stayin)+".txt", "a+")
            for data in vector_info:
                f.write(str(data) + " ")
            f.write("\n")
            f.close()
            vector_info_global = [act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t]
            f = open("./data/info_vuelo.txt", "a+")
            for data in vector_info_global:
                f.write(str(data) + " ")
            f.write("\n")
            f.close()'''

            rate.sleep()

        if disconnect:
            self._result.final_pose.position.x = self._feedback.act_pose.position.x
            self._result.final_pose.position.y = self._feedback.act_pose.position.y
            self._result.final_pose.position.z = self._feedback.act_pose.position.z
            self._result.final_pose.orientation.z = self._feedback.act_pose.orientation.z
            print ('Drone disconnected.')
            self.server.set_succeeded(self._result)
        # NO SE SI SE PODRIA QUITAR
        elif success:
            self._result.final_pose.position.x = self._feedback.act_pose.position.x
            self._result.final_pose.position.y = self._feedback.act_pose.position.y
            self._result.final_pose.position.z = self._feedback.act_pose.position.z
            self._result.final_pose.orientation.z = self._feedback.act_pose.orientation.z
            print ('Action cancelled. Sending drone position')
            self.server.set_succeeded(self._result)


class PathFollowingServer(object):
    # create messages that are used to publish feedback/result
    _feedback = control_parrot.msg.PathFollowingFeedback()
    _result = control_parrot.msg.PathFollowingResult()
    def __init__(self, name):
        self.server_name = name
        self.server = actionlib.SimpleActionServer(self.server_name, control_parrot.msg.PathFollowingAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        global drone_pose, drone_yaw, drone_twist, state, drone_accel
        global flag_action, client, last_position
        mutex.acquire()
        flag = flag_action
        mutex.release()
        if flag == D_ACTION_STAYIN:
            client.cancel_goal()
        mutex.acquire()
        flag_action = D_ACTION_ON
        mutex.release()
        success = True
        disconnect = False

        # SEGUIMIENTO DE LINEA

        if goal.mode == 1:
            # parametros
            thr = 0.05
            k = 1

            kp_vax = 0.7
            kp_vay = 0.7
            kd_vax = 0.3
            kd_vay = 0.3


            a0 = rospy.get_param('~param_a0', 0)
            a0 = float(a0)
            a1 = rospy.get_param('~param_a1', 0)
            a1 = float(a1)            
            a2 = rospy.get_param('~param_a2', 0)
            a2 = float(a2)


            # extraer info
            global cont_accion_path_line
            cont_accion_path_line = cont_accion_path_line + 1

            W1 = np.array(goal.w1.data)
            W2 = np.array(goal.w2.data)
            Va_des = np.array(goal.va)
            # va_des puede ir de 0,1 a 4

            mutex.acquire()
            s = state
            act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
            act_t = drone_pose.header.stamp.to_sec()
            act_yaw = drone_yaw
            act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y, drone_twist.twist.linear.z,
                       drone_twist.twist.angular.z]
            act_accel = [drone_accel.accel.linear.x, drone_accel.accel.linear.y, drone_accel.accel.linear.z]
            self._feedback.act_pose.position.x = act_pose[0]
            self._feedback.act_pose.position.y = act_pose[1]
            self._feedback.act_pose.position.z = act_pose[2]
            self._feedback.act_pose.orientation.z = act_yaw
            mutex.release()

            # impulso inicial (con un Va = 10porciento) CARROT CHASING ALGORITHM
            Ru = np.linalg.norm([W1[0] - act_pose[0], W1[1] - act_pose[1]])
            tetha = np.arctan2(W2[1] - W1[1], W2[0] - W1[0])
            tethau = np.arctan2(act_pose[1] - W1[1], act_pose[0] - W1[0])
            beta = tetha - tethau
            R = math.sqrt(math.pow(Ru, 2) - math.pow(Ru * math.sin(beta), 2))
            S = [(R) * math.cos(tetha) + W1[0], (R) * math.sin(tetha) + W1[1]]
            phi_init = np.arctan2(S[1] - act_pose[1], S[0] - act_pose[0])
            cmd_x = 10*math.cos(phi_init)
            cmd_y = 10*math.sin(phi_init)
            vx = math.cos(act_yaw) * cmd_x + math.sin(act_yaw) * cmd_y
            vy = math.sin(act_yaw) * cmd_x - math.cos(act_yaw) * cmd_y
            drone.move(vx, vy, 0, 0)

            rospy.loginfo('Drone following a straight-line path. W1=[' + str(W1[0]) + ', ' + str(
                W1[1]) + '] W2=[' + str(W2[0]) + ', ' + str(W2[1]) + ']')

            rate = rospy.Rate(15)  # 15hz
            while not rospy.is_shutdown():
                # check that preempt has not been requested by the client
                if self.server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self.server_name)
                    self.server.set_preempted()
                    success = False
                    break
                if s != S_CONNECTED:
                    rospy.loginfo('Drone disconnected')
                    disconnect = True
                    break
                # refresh values
                mutex.acquire()
                act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
                act_t = drone_pose.header.stamp.to_sec()
                act_yaw = drone_yaw
                act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y, drone_twist.twist.linear.z,
                           drone_twist.twist.angular.z]
                act_accel = [drone_accel.accel.linear.x, drone_accel.accel.linear.y, drone_accel.accel.linear.z]
                mutex.release()
                # distance = p_final-act_pose (ALGO ASI)
                self._feedback.act_pose.position.x = act_pose[0]
                self._feedback.act_pose.position.y = act_pose[1]
                self._feedback.act_pose.position.z = act_pose[2]
                self._feedback.act_pose.orientation.z = act_yaw
                # publish the feedback
                self.server.publish_feedback(self._feedback)

                # if distance < thr:
                    # break

                # CARROT CHASING ALGORITHM
                # Control path following (calculo de phi_des)

                '''act_phi = np.arctan2(act_vel[1], act_vel[0])
                act_Va = np.linalg.norm([act_vel[0], act_vel[1]])
                Ru = np.linalg.norm([W1[0]-act_pose[0], W1[1]-act_pose[1]])
                tetha = np.arctan2(W2[1]-W1[1], W2[0]-W1[0])
                tethau = np.arctan2(act_pose[1]-W1[1], act_pose[0]-W1[0])
                beta = tetha - tethau
                R = math.sqrt(math.pow(Ru, 2)-math.pow(Ru*math.sin(beta),2))
                # d=cross track error y puede ser negativo
                d = Ru*math.sin(beta)
                # dist es siempre positivo. Se utiliza como variable proporcional para sacar delta
                dist = abs(d)
                delta = a0+a1*dist+a2*dist**2
                delta = np.clip(delta, 0, 100)
                S = [(R+delta) * math.cos(tetha) + W1[0], (R+delta) * math.sin(tetha) + W1[1]]
                des_phi = np.arctan2(S[1]-act_pose[1], S[0]-act_pose[0])
                print ('New measure follow line')
                print ("Va teorico: " + str(Va_des))
                print ("Va real: " + str(act_Va) + " phi real: " + str(act_phi))
                print ("Cross-track error: " + str(d))
                u = k * (des_phi - act_phi)
                phi_des = act_phi + u'''


                # NUEVA FORMA DE PENSAR
                act_phi = np.arctan2(act_vel[1], act_vel[0])
                act_Va = np.linalg.norm([act_vel[0], act_vel[1]])

                Ru_vect = act_pose[0:2] - W1
                R_vect = W2 - W1
                R_norm = np.linalg.norm(R_vect)
                d = abs(np.cross(Ru_vect, R_vect) / (R_norm + 1e-6))
                R = abs(np.dot(Ru_vect, R_vect) / (R_norm + 1e-6))
                delta = a0 + a1 * d + a2 * d ** 2
                delta = np.clip(delta, 0, 100)
                S = W1 + (R + delta) * R_vect / R_norm
                phi_des = np.arctan2(S[1] - act_pose[1], S[0] - act_pose[0])
                # print ('New measure follow line')
                # print ("Va teorico: " + str(Va_des))
                # print ("Va real: " + str(act_Va) + " phi real: " + str(act_phi))
                print ("Cross-track error: " + str(d))
                print("Posicion: " + str(act_pose))
                print("R: " + str(R))
                print("delta" + str(delta))
                print("S: " + str(S))
                print("Phi deseado: " + str(phi_des))

                # CONTROL EN VELOCIDAD (necesito phi_des y Va_des en m/s)
                # proyeccion en x e y de phi_des
                phi_des_x = math.cos(phi_des)
                phi_des_y = math.sin(phi_des)

                Vax_des = Va_des * phi_des_x
                Vay_des = Va_des * phi_des_y

                pdx_va = (Vax_des - act_vel[0]) * kp_vax + act_accel[0] * kd_vax
                pdy_va = (Vay_des - act_vel[1]) * kp_vay + act_accel[1] * kd_vay

                # anadimos a cmd_vel el pd
                Vax_des_pd = Vax_des + pdx_va
                Vay_des_pd = Vay_des + pdy_va

                # pasamos de m/s a porcentaje
                coefx = [-0.000668940781647, -0.0326762435076, -0.0294549051165, 16.1544748879, 0.162490399212]
                coefy = [0.00687112729217, -0.014779740721, -0.11999831192, 18.1045243274, -0.0331072136715]
                polx = np.poly1d(coefx)
                poly = np.poly1d(coefy)
                Vax_des_porcent = polx(Vax_des_pd)
                Vay_des_porcent = poly(Vay_des_pd)

                act_phi = np.arctan2(act_vel[1], act_vel[0])
                act_Va = np.linalg.norm([act_vel[0], act_vel[1]])
                act_phi_x = math.cos(act_phi)
                act_phi_y = math.sin(act_phi)

                # comando de movimiento con control pd
                cmd_x = Vax_des_porcent
                cmd_y = Vay_des_porcent
                cmd_va = np.linalg.norm([cmd_x, cmd_y])
                print ("Linea recta")
                print ("cmd_x: " + str(cmd_x) + " cmd_y: " + str(cmd_y))
                vx = math.cos(act_yaw) * cmd_x + math.sin(act_yaw) * cmd_y
                vy = math.sin(act_yaw) * cmd_x - math.cos(act_yaw) * cmd_y
                np.clip(vx, -80, 80)
                np.clip(vy, -80, 80)
                print ("vx_cmd: " + str(vx) + " vy_cmd: " + str(vy))
                cmd_va_drone = np.linalg.norm([vx, vy])

                drone.move(vx, vy, 0, 0)


                # SALVAR EN TXT VARIABLES
                vector_info = [d, d, a0, a1, a2, W1[0], W1[1], W2[0], W2[1], act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t, cmd_x, cmd_y, act_vel[0], act_vel[1], act_vel[2], act_vel[3], act_accel[0], act_accel[1], act_accel[2], act_Va, Va_des, Vax_des, Vay_des, cmd_va, cmd_va_drone, act_phi, phi_des, act_phi_x, act_phi_y, phi_des_x, phi_des_y, delta]
                '''f = open("./data/info_drone_path_line"+str(cont_accion_path_line)+".txt", "a+")
                for data in vector_info:
                    f.write(str(data)+" ")
                f.write("\n")
                f.close()
                vector_info_global = [act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t]
                f = open("./data/info_vuelo.txt", "a+")
                for data in vector_info_global:
                    f.write(str(data) + " ")
                f.write("\n")
                f.close()'''

                '''f = open("./data/info_drone_"+str(a0)+"_"+str(a1)+"_"+str(a2)+".txt", "a+")
                for data in vector_info:
                    f.write(str(data)+" ")
                f.write("\n")
                f.close()'''

                rate.sleep()


        # SEGUIMIENTO DE CIRCULO

        elif goal.mode == 0:
            # parametros
            lamda = 0.2
            k = 1

            kp_vax = 0.7
            kp_vay = 0.7
            kd_vax = 0.3
            kd_vay = 0.3

            # extraer info
            global cont_accion_path_loiter
            cont_accion_path_loiter = cont_accion_path_loiter + 1

            O = [goal.w1.data[0], goal.w1.data[1]]
            r = goal.w2.data[0]
            Va_des = goal.va

            mutex.acquire()
            s = state
            act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
            act_t = drone_pose.header.stamp.to_sec()
            act_yaw = drone_yaw
            act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y, drone_twist.twist.linear.z,
                       drone_twist.twist.angular.z]
            act_accel = [drone_accel.accel.linear.x, drone_accel.accel.linear.y, drone_accel.accel.linear.z]
            self._feedback.act_pose.position.x = act_pose[0]
            self._feedback.act_pose.position.y = act_pose[1]
            self._feedback.act_pose.position.z = act_pose[2]
            self._feedback.act_pose.orientation.z = act_yaw
            mutex.release()

            # impulso inicial (con un Va = 10porciento) CARROT CHASING ALGORITHM
            d = np.linalg.norm([O[0] - act_pose[0], O[1] - act_pose[1]]) - r
            tetha = np.arctan2(act_pose[1] - O[1], act_pose[0] - O[0])
            S = [r * math.cos(tetha+lamda) + O[0], r * math.sin(tetha+lamda) + O[1]]
            phi_init = np.arctan2(S[1] - act_pose[1], S[0] - act_pose[0])
            cmd_x = 10*math.cos(phi_init)
            cmd_y = 10*math.sin(phi_init)
            vx = math.cos(act_yaw) *cmd_x + math.sin(act_yaw) * cmd_y
            vy = math.sin(act_yaw) * cmd_x - math.cos(act_yaw) * cmd_y
            drone.move(vx, vy, 0, 0)

            rospy.loginfo('Drone following a loiter path. O=[' + str(O[0]) + ', ' + str(
                O[1]) + '] r=' + str(r))

            rate = rospy.Rate(15)  # 15hz
            while not rospy.is_shutdown():
                # check that preempt has not been requested by the client
                if self.server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self.server_name)
                    self.server.set_preempted()
                    success = False
                    break
                if s != S_CONNECTED:
                    rospy.loginfo('Drone disconnected')
                    disconnect = True
                    break
                # refresh values
                mutex.acquire()
                act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
                act_t = drone_pose.header.stamp.to_sec()
                act_yaw = drone_yaw
                act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y, drone_twist.twist.linear.z,
                           drone_twist.twist.angular.z]
                act_accel = [drone_accel.accel.linear.x, drone_accel.accel.linear.y, drone_accel.accel.linear.z]
                mutex.release()
                # distance = p_final-act_pose (ALGO ASI)
                self._feedback.act_pose.position.x = act_pose[0]
                self._feedback.act_pose.position.y = act_pose[1]
                self._feedback.act_pose.position.z = act_pose[2]
                self._feedback.act_pose.orientation.z = act_yaw
                # publish the feedback
                self.server.publish_feedback(self._feedback)

                # if distance < thr:
                    # break

                # CARROT CHASING ALGORITHM
                act_phi = np.arctan2(act_vel[1], act_vel[0])
                act_Va = np.linalg.norm([act_vel[0], act_vel[1]])
                d = np.linalg.norm([O[0] - act_pose[0], O[1] - act_pose[1]]) - r
                tetha = np.arctan2(act_pose[1] - O[1], act_pose[0] - O[0])
                S = [r * math.cos(tetha + lamda) + O[0], r * math.sin(tetha + lamda) + O[1]]
                des_phi = np.arctan2(S[1]-act_pose[1], S[0]-act_pose[0])
                print ('New measure follow loiter')
                print ("Va teorico: " + str(Va_des))
                print ("Va real: " + str(act_Va) + " phi real: " + str(act_phi))
                print ("Cross-track error: " + str(d))
                u = k * (des_phi - act_phi)
                phi_des = act_phi + u

                # CONTROL EN VELOCIDAD (necesito phi_des y Va_des en m/s)
                # proyeccion en x e y de phi_des
                phi_des_x = math.cos(phi_des)
                phi_des_y = math.sin(phi_des)

                Vax_des = Va_des * phi_des_x
                Vay_des = Va_des * phi_des_y

                pdx_va = (Vax_des - act_vel[0]) * kp_vax + act_accel[0] * kd_vax
                pdy_va = (Vay_des - act_vel[1]) * kp_vay + act_accel[1] * kd_vay

                # anadimos a cmd_vel el pd
                Vax_des_pd = Vax_des + pdx_va
                Vay_des_pd = Vay_des + pdy_va

                # pasamos de m/s a porcentaje
                coefx = [-0.000668940781647, -0.0326762435076, -0.0294549051165, 16.1544748879, 0.162490399212]
                coefy = [0.00687112729217, -0.014779740721, -0.11999831192, 18.1045243274, -0.0331072136715]
                polx = np.poly1d(coefx)
                poly = np.poly1d(coefy)
                Vax_des_porcent = polx(Vax_des_pd)
                Vay_des_porcent = poly(Vay_des_pd)

                act_phi = np.arctan2(act_vel[1], act_vel[0])
                act_Va = np.linalg.norm([act_vel[0], act_vel[1]])
                act_phi_x = math.cos(act_phi)
                act_phi_y = math.sin(act_phi)

                # comando de movimiento con control pd
                cmd_x = Vax_des_porcent
                cmd_y = Vay_des_porcent
                cmd_va = np.linalg.norm([cmd_x, cmd_y])
                print ("Linea recta")
                print ("cmd_x: " + str(cmd_x) + " cmd_y: " + str(cmd_y))
                vx = math.cos(act_yaw) * cmd_x + math.sin(act_yaw) * cmd_y
                vy = math.sin(act_yaw) * cmd_x - math.cos(act_yaw) * cmd_y
                np.clip(vx, -80, 80)
                np.clip(vy, -80, 80)
                print ("vx_cmd: " + str(vx) + " vy_cmd: " + str(vy))
                cmd_va_drone = np.linalg.norm([vx, vy])

                drone.move(vx, vy, 0, 0)

                # SALVAR EN TXT VARIABLES
                vector_info = [act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t, cmd_x, cmd_y, act_vel[0], act_vel[1], act_vel[2], act_vel[3], act_accel[0], act_accel[1], act_accel[2], act_Va, Va_des, Vax_des, Vay_des, cmd_va, cmd_va_drone, act_phi, phi_des, act_phi_x, act_phi_y, phi_des_x, phi_des_y]
                f = open("./data/info_drone_path_loiter"+str(cont_accion_path_loiter)+".txt", "a+")
                for data in vector_info:
                    f.write(str(data) + " ")
                f.write("\n")
                f.close()
                vector_info_global = [act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t]
                f = open("./data/info_vuelo.txt", "a+")
                for data in vector_info_global:
                    f.write(str(data) + " ")
                f.write("\n")
                f.close()

                rate.sleep()


        # SEGUIMIENTO DE WAYPOINTS

        elif goal.mode == 2:
            # parametros
            # definir parametros

            kp_vax = 0.7
            kp_vay = 0.7
            kd_vax = 0.3
            kd_vay = 0.3

            # extraer info
            global cont_accion_path_wp
            cont_accion_path_wp = cont_accion_path_wp + 1

            Va_des = goal.va
            spline_type = goal.w1.data[0]
            wp_matrix = zip(goal.w2.data[::2], goal.w2.data[1::2])
            num_wp = len(wp_matrix)

            mutex.acquire()
            s = state
            act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
            act_t = drone_pose.header.stamp.to_sec()
            act_yaw = drone_yaw
            act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y, drone_twist.twist.linear.z,
                       drone_twist.twist.angular.z]
            act_accel = [drone_accel.accel.linear.x, drone_accel.accel.linear.y, drone_accel.accel.linear.z]
            self._feedback.act_pose.position.x = act_pose[0]
            self._feedback.act_pose.position.y = act_pose[1]
            self._feedback.act_pose.position.z = act_pose[2]
            self._feedback.act_pose.orientation.z = act_yaw
            mutex.release()


            # anadir impulso inicial en direccion al primer wp

            rospy.loginfo('Drone following a path given by ' + str(num_wp) + ' waypoints with a spline of order ' + str(spline_type))

            rate = rospy.Rate(15)  # 15hz
            while not rospy.is_shutdown():
                # check that preempt has not been requested by the client
                if self.server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self.server_name)
                    self.server.set_preempted()
                    success = False
                    break
                if s != S_CONNECTED:
                    rospy.loginfo('Drone disconnected')
                    disconnect = True
                    break
                # refresh values
                mutex.acquire()
                act_pose = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
                act_t = drone_pose.header.stamp.to_sec()
                act_yaw = drone_yaw
                act_vel = [drone_twist.twist.linear.x, drone_twist.twist.linear.y, drone_twist.twist.linear.z,
                           drone_twist.twist.angular.z]
                act_accel = [drone_accel.accel.linear.x, drone_accel.accel.linear.y, drone_accel.accel.linear.z]
                mutex.release()
                # distance = p_final-act_pose (ALGO ASI)
                self._feedback.act_pose.position.x = act_pose[0]
                self._feedback.act_pose.position.y = act_pose[1]
                self._feedback.act_pose.position.z = act_pose[2]
                self._feedback.act_pose.orientation.z = act_yaw
                # publish the feedback
                self.server.publish_feedback(self._feedback)

                # if distance < thr:
                    # break

                # NLGL ALGORITHM
                # anadir algoritmo NLGL
                Va_des = 1.0
                phi_des = 1.57

                # CONTROL EN VELOCIDAD (necesito phi_des y Va_des en m/s)
                # proyeccion en x e y de phi_des
                phi_des_x = math.cos(phi_des)
                phi_des_y = math.sin(phi_des)

                Vax_des = Va_des * phi_des_x
                Vay_des = Va_des * phi_des_y

                pdx_va = (Vax_des - act_vel[0]) * kp_vax + act_accel[0] * kd_vax
                pdy_va = (Vay_des - act_vel[1]) * kp_vay + act_accel[1] * kd_vay

                # anadimos a cmd_vel el pd
                Vax_des_pd = Vax_des + pdx_va
                Vay_des_pd = Vay_des + pdy_va

                # pasamos de m/s a porcentaje
                coefx = [-0.000668940781647, -0.0326762435076, -0.0294549051165, 16.1544748879, 0.162490399212]
                coefy = [0.00687112729217, -0.014779740721, -0.11999831192, 18.1045243274, -0.0331072136715]
                polx = np.poly1d(coefx)
                poly = np.poly1d(coefy)
                Vax_des_porcent = polx(Vax_des_pd)
                Vay_des_porcent = poly(Vay_des_pd)

                act_phi = np.arctan2(act_vel[1], act_vel[0])
                act_Va = np.linalg.norm([act_vel[0], act_vel[1]])
                act_phi_x = math.cos(act_phi)
                act_phi_y = math.sin(act_phi)

                # comando de movimiento con control pd
                cmd_x = Vax_des_porcent
                cmd_y = Vay_des_porcent
                cmd_va = np.linalg.norm([cmd_x, cmd_y])
                print ("Linea recta")
                print ("cmd_x: " + str(cmd_x) + " cmd_y: " + str(cmd_y))
                vx = math.cos(act_yaw) * cmd_x + math.sin(act_yaw) * cmd_y
                vy = math.sin(act_yaw) * cmd_x - math.cos(act_yaw) * cmd_y
                np.clip(vx, -80, 80)
                np.clip(vy, -80, 80)
                print ("vx_cmd: " + str(vx) + " vy_cmd: " + str(vy))
                cmd_va_drone = np.linalg.norm([vx, vy])

                drone.move(vx, vy, 0, 0)

                # SALVAR EN TXT VARIABLES
                vector_info = [act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t, cmd_x, cmd_y, act_vel[0], act_vel[1], act_vel[2], act_vel[3], act_accel[0], act_accel[1], act_accel[2], act_Va, Va_des, Vax_des, Vay_des, cmd_va, cmd_va_drone, act_phi, phi_des, act_phi_x, act_phi_y, phi_des_x, phi_des_y]
                f = open("./data/info_drone_path_wp"+str(cont_accion_path_wp)+".txt", "a+")
                for data in vector_info:
                    f.write(str(data) + " ")
                f.write("\n")
                f.close()
                vector_info_global = [act_pose[0], act_pose[1], act_pose[2], act_yaw, act_t]
                f = open("./data/info_vuelo.txt", "a+")
                for data in vector_info_global:
                    f.write(str(data) + " ")
                f.write("\n")
                f.close()

                rate.sleep()


        if disconnect:
            self._result.final_pose.position.x = self._feedback.act_pose.position.x
            self._result.final_pose.position.y = self._feedback.act_pose.position.y
            self._result.final_pose.position.z = self._feedback.act_pose.position.z
            self._result.final_pose.orientation.z = self._feedback.act_pose.orientation.z
            print ('Exit of action')
            self.server.set_succeeded(self._result)
        elif success:
            self._result.final_pose.position.x = self._feedback.act_pose.position.x
            self._result.final_pose.position.y = self._feedback.act_pose.position.y
            self._result.final_pose.position.z = self._feedback.act_pose.position.z
            self._result.final_pose.orientation.z = self._feedback.act_pose.orientation.z
            print ('Arrival to waypoint succeeded')
            self.server.set_succeeded(self._result)

        mutex.acquire()
        last_position = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z, drone_yaw]
        flag_action = D_FLYINGFREE
        mutex.release()


# DEFINICION DE CLIENTS DE ACCIONES (acciones que se llaman en el propio nodo)

def land_client():
    rospy.loginfo("Action call: Land")
    clientl = actionlib.SimpleActionClient('/drone' + str(num_drone) + '/land', control_parrot.msg.LandAction)
    clientl.wait_for_server()
    goal = control_parrot.msg.LandGoal(goal=1)
    clientl.send_goal(goal)
    clientl.wait_for_result()
    return clientl.get_result()


def goto_waypoint_client(x_des, y_des, z_des, yaw_des):
    rospy.loginfo("Action call: Go to Waypoint [" + str(x_des) + ', ' + str(y_des) + ', ' + str(z_des) + ']  with yaw = ' + str(yaw_des))
    clientg = actionlib.SimpleActionClient('/drone' + str(num_drone) + '/goto_waypoint', control_parrot.msg.GotoWaypointAction)
    clientg.wait_for_server()
    goal = control_parrot.msg.GotoWaypointGoal()
    goal.des_pose.position.x = x_des
    goal.des_pose.position.y = y_des
    goal.des_pose.position.z = z_des
    goal.des_pose.orientation.z = yaw_des
    clientg.send_goal(goal)
    clientg.wait_for_result()
    return clientg.get_result()


# DEFINICION DE CLIENTES DE SERVICIOS (servicios que se llaman en el propio nodo)

def change_mode_client(mode):
    rospy.loginfo("Service call: Change mode")
    rospy.wait_for_service('/drone' + str(num_drone) + '/change_mode')
    try:
        change_mode = rospy.ServiceProxy('/drone' + str(num_drone) + '/change_mode', ChangeModeService)
        resp1 = change_mode(mode)
        return resp1.msg
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# DEFINICION DE MAIN

if __name__ == '__main__':
    global drone, state, message, config, speed, battery
    global drone_pose, drone_twist, drone_accel, drone_yaw
    global flag_action, last_position
    global client
    global cont_accion_land, cont_accion_takeoff, cont_accion_goto, cont_accion_stayin, cont_accion_path_line, cont_accion_path_loiter, cont_accion_path_wp
    global flag_control_mode

    # 0=comando por acciones // 1=comando por velocidades
    flag_control_mode = 0

    cont_accion_land = 0
    cont_accion_takeoff = 0
    cont_accion_goto = 0
    cont_accion_stayin = 0
    cont_accion_path_line = 0
    cont_accion_path_loiter = 0
    cont_accion_path_wp = 0

    flag_action = D_GROUND
    last_position = [0.0, 0.0, 0.0, 0.0]
    last_pose = [0.0, 0.0, 0.0, 0.0]

    drone_yaw = 0
    drone_pose = PoseStamped()
    drone_twist = TwistStamped()
    drone_accel = AccelStamped()
    state = S_DISCONNECTED
    message = speed = battery = ''
    config = dict()
    drone = minidrone.MiniDrone(mac=DRONEMAC, callback=refresh_data)


    #subscriber
    rospy.Subscriber('/drone' + str(num_drone) + '/position', PoseStamped, pose_callback)
    rospy.Subscriber('/drone' + str(num_drone) + '/twist', TwistStamped, twist_callback)
    rospy.Subscriber('/drone' + str(num_drone) + '/accel', AccelStamped, accel_callback)

    rospy.Subscriber('/drone' + str(num_drone) + '/cmd_incr_vel', Twist, cmd_incr_vel_callback)
    rospy.Subscriber('/drone' + str(num_drone) + '/cmd_velocity', Twist, cmd_velocity_callback)

    #publisher
    pub_state = rospy.Publisher('/drone' + str(num_drone) + '/state', Int16, queue_size=10)
    pub_battery = rospy.Publisher('/drone' + str(num_drone) + '/battery', Int32, queue_size=10)
    #services
    srv_disconnect = rospy.Service('/drone' + str(num_drone) + '/disconnect', DroneService, handle_disconnect)
    srv_increase = rospy.Service('/drone' + str(num_drone) + '/increase', DroneService, handle_increase)
    srv_decrease = rospy.Service('/drone' + str(num_drone) + '/decrease', DroneService, handle_decrease)
    srv_emergency = rospy.Service('/drone' + str(num_drone) + '/emergency', DroneService, handle_emergency)
    srv_change_mode = rospy.Service('/drone' + str(num_drone) + '/change_mode', ChangeModeService, handle_change_mode)


    #actions
    server_connect = ConnectServer('/drone' + str(num_drone) + '/connect')
    server_takeoff = TakeOffServer('/drone' + str(num_drone) + '/take_off')
    server_land = LandServer('/drone' + str(num_drone) + '/land')
    server_gotowaypoint = GotoWaypointServer('/drone' + str(num_drone) + '/goto_waypoint')
    server_stayinwaypoint = StayinWaypointServer('/drone' + str(num_drone) + '/stayin_waypoint')
    server_pathfollowing = PathFollowingServer('/drone' + str(num_drone) + '/path_following')

    print ("CONTROL OF THE DRONE \n")

    rate = rospy.Rate(30)  # 30hz
    while not rospy.is_shutdown():
        mutex.acquire()
        s = state
        flag = flag_action
        b = battery
        mutex.release()
        # publish state
        pub_state.publish(s)
        flag_bat_ok = 1
        if b == '':
            flag_bat_ok = 0
        if s == S_CONNECTED and flag_bat_ok == 1:
            # publish battery
            pub_battery.publish(int(b))
            # retorno automatico a la base por bateria
            if int(b) < 5 and int(b) != 0:
                if flag_action != D_EMERGENCY and flag_action != D_CHANGEBAT:
                    print ('EMERGENCY: LOW BATTERY')
                    if flag == D_ACTION_STAYIN:
                        client.cancel_goal()
                        flag_action = D_EMERGENCY
                    if flag == D_GROUND:
                        print ('Drone landed')
                        flag_action = D_CHANGEBAT
                    if flag == D_ACTION_ON:
                        print ('Waiting for an action to be finished to return to base')
                if flag_action == D_EMERGENCY:
                    result = goto_waypoint_client(0.0, 0.0, 0.75, 0.0)
                    print ('Waypoint achieved (Pose = [' + str(result.final_pose.position.x) + ', ' + str(result.final_pose.position.y) + ', ' + str(result.final_pose.position.z) + ']  yaw = ' + str(result.final_pose.orientation.z))
                    result = land_client()
                    print ('Drone landed (z = ' + str(result.final_z) + ')')
                    flag_action = D_CHANGEBAT
                if flag == D_CHANGEBAT:
                    print ('DRONE IN BASE POINT. CHANGE BATTERY')

        # Mantenimiento en la ultima posicion (llamar a stayin_waypoint) si no se manda ninguna accion
        if flag_control_mode==0 and flag == D_FLYINGFREE:
            mutex.acquire()
            last_pose[0] = last_position[0]
            last_pose[1] = last_position[1]
            last_pose[2] = last_position[2]
            last_pose[3] = last_position[3]
            mutex.release()
            print ("Action call: Stay in last position [" + str(last_pose[0]) + ', ' + str(last_pose[1]) + ', ' + str(last_pose[2]) + ']  with yaw = ' + str(last_pose[3]))
            client = actionlib.SimpleActionClient('/drone' + str(num_drone) + '/stayin_waypoint', control_parrot.msg.GotoWaypointAction)
            client.wait_for_server()
            goal = control_parrot.msg.GotoWaypointGoal()
            goal.des_pose.position.x = last_pose[0]
            goal.des_pose.position.y = last_pose[1]
            goal.des_pose.position.z = last_pose[2]
            goal.des_pose.orientation.z = last_pose[3]
            client.send_goal(goal)
            mutex.acquire()
            flag_action = D_ACTION_STAYIN
            mutex.release()
        rate.sleep()



    drone.land()
    time.sleep(5)
    drone.die()

