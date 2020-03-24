#!/usr/bin/env python

import rospy
import threading
import roslib
roslib.load_manifest('control_parrot')
import actionlib
import control_parrot.msg
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from control_parrot.srv import *

rospy.init_node('client')

num_drone = rospy.get_param('~param_num', 0)
num_drone = int(num_drone)


S_DISCONNECTED = 0
S_CONNECTING = 1
S_CONNECTED = 2

mutex = threading.Lock()

# cmd_keyboard

class _GetchUnix:
    def __init__(self):
        import tty, sys, termios # import termios now or else you'll get the Unix version on the Mac

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# callback

def state_callback(msg):
    global state
    if msg.data == S_DISCONNECTED:
        mutex.acquire()
        state = S_DISCONNECTED
        mutex.release()
    if msg.data == S_CONNECTING:
        mutex.acquire()
        state = S_CONNECTING
        mutex.release()
    if msg.data == S_CONNECTED:
        mutex.acquire()
        state = S_CONNECTED
        mutex.release()

# clients de servicios

def change_mode_client(mode):
    rospy.loginfo("Service call: Change mode")
    rospy.wait_for_service('/drone' + str(num_drone) + '/change_mode')
    try:
        change_mode = rospy.ServiceProxy('/drone' + str(num_drone) + '/change_mode', ChangeModeService)
        resp1 = change_mode(mode)
        return resp1.msg
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def disconnect_client():
    rospy.wait_for_service('/drone' + str(num_drone) + '/disconnect')
    try:
        disconnect = rospy.ServiceProxy('/drone' + str(num_drone) + '/disconnect', DroneService)
        resp1 = disconnect()
        return resp1.msg
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def increase_client():
    rospy.wait_for_service('/drone' + str(num_drone) + '/increase')
    try:
        increase = rospy.ServiceProxy('/drone' + str(num_drone) + '/increase', DroneService)
        resp1 = increase()
        return resp1.msg
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def decrease_client():
    rospy.wait_for_service('/drone' + str(num_drone) + '/decrease')
    try:
        decrease = rospy.ServiceProxy('/drone' + str(num_drone) + '/decrease', DroneService)
        resp1 = decrease()
        return resp1.msg
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def emergency_client():
    rospy.wait_for_service('/drone' + str(num_drone) + '/emergency')
    try:
        emergency = rospy.ServiceProxy('/drone' + str(num_drone) + '/emergency', DroneService)
        resp1 = emergency()
        return resp1.msg
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

# clients de acciones

def connect_client():
    rospy.loginfo("Action call: Connect")
    client = actionlib.SimpleActionClient('/drone' + str(num_drone) + '/connect', control_parrot.msg.ConnectAction)
    client.wait_for_server()
    goal = control_parrot.msg.ConnectGoal(goal=1)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


def take_off_client():
    rospy.loginfo("Action call: Take off")
    client = actionlib.SimpleActionClient('/drone' + str(num_drone) + '/take_off', control_parrot.msg.TakeOffAction)
    client.wait_for_server()
    goal = control_parrot.msg.TakeOffGoal(goal=1)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


def land_client():
    rospy.loginfo("Action call: Land")
    client = actionlib.SimpleActionClient('/drone' + str(num_drone) + '/land', control_parrot.msg.LandAction)
    client.wait_for_server()
    goal = control_parrot.msg.LandGoal(goal=1)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


def stayin_waypoint_client(x_des, y_des, z_des, yaw_des):
    rospy.loginfo("Action call: Go and stay in Waypoint [" + str(x_des) + ', ' + str(y_des) + ', ' + str(z_des) + ']  with yaw = ' + str(yaw_des))
    client = actionlib.SimpleActionClient('/drone' + str(num_drone) + '/stayin_waypoint', control_parrot.msg.GotoWaypointAction)
    client.wait_for_server()
    goal = control_parrot.msg.GotoWaypointGoal()
    goal.des_pose.position.x = x_des
    goal.des_pose.position.y = y_des
    goal.des_pose.position.z = z_des
    goal.des_pose.orientation.z = yaw_des
    client.send_goal(goal)
    client.wait_for_result()
    # client.cancel_goal() # cancels the goal currently pursuing
    return client.get_result()


def goto_waypoint_client(x_des, y_des, z_des, yaw_des):
    rospy.loginfo("Action call: Go to Waypoint [" + str(x_des) + ', ' + str(y_des) + ', ' + str(z_des) + ']  with yaw = ' + str(yaw_des))
    client = actionlib.SimpleActionClient('/drone' + str(num_drone) + '/goto_waypoint', control_parrot.msg.GotoWaypointAction)
    client.wait_for_server()
    goal = control_parrot.msg.GotoWaypointGoal()
    goal.des_pose.position.x = x_des
    goal.des_pose.position.y = y_des
    goal.des_pose.position.z = z_des
    goal.des_pose.orientation.z = yaw_des
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


def path_following_client(mode, w1, w2, v):
    if mode == 0:
        rospy.loginfo("Action call: Follow a circular path. 0 = [" + str(w1[0]) + ', ' + str(w1[1]) + '] R = ' + str(w2[0]) + '  V = ' + str(v))
    if mode == 1:
        rospy.loginfo("Action call: Follow a straight-line path. W1=[" + str(w1[0]) + ', ' + str(w1[1]) + '] W2=[' + str(w2[0]) + ', ' + str(w2[1])+']  V = ' + str(v))
    if mode == 2:
        num_waypoints = len(w2)/2
        spline_type = w1
        rospy.loginfo("Action call: Follow a path commanded by " +str(num_waypoints)+ " waypoints with a spline of order " + str(spline_type))

    client = actionlib.SimpleActionClient('/drone' + str(num_drone) + '/path_following', control_parrot.msg.PathFollowingAction)
    client.wait_for_server()
    goal = control_parrot.msg.PathFollowingGoal()
    goal.mode = mode
    goal.w1.data = w1
    goal.w2.data = w2
    goal.va = v
    client.send_goal(goal)
    client.wait_for_result()
    # client.cancel_goal() # cancels the goal currently pursuing
    return client.get_result()




# ejemplos funciones client

def cmd_keyboard():
    global drone_speed
    global state

    pub_incr_vel = rospy.Publisher('/drone' + str(num_drone) + '/cmd_incr_vel', Twist, queue_size=10)
    incr_vel_msg = Twist()
    incr_vel_msg.linear.x = 0
    incr_vel_msg.linear.y = 0
    incr_vel_msg.linear.z = 0
    incr_vel_msg.angular.x = 0
    incr_vel_msg.angular.y = 0
    incr_vel_msg.angular.z = 0

    pub_velocity = rospy.Publisher('/drone' + str(num_drone) + '/cmd_velocity', Twist, queue_size=10)
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    velocity_msg.linear.z = 0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0
    velocity_msg.angular.z = 0

    rate = rospy.Rate(10) # 10hz
    print ("CONTROL DRONE MODE: COMMAND KEYBOARD \n")
    while not rospy.is_shutdown():
        mutex.acquire()
        s = state
        mutex.release()

        if s == S_DISCONNECTED:
            print ("Press: 'c' to connect")
            getch1 = _GetchUnix()
            key = getch1()
            # cmd_connect
            if key == 'c':
                result = connect_client()
                print ('Drone connected (state = ' + str(result.result) + ')')
                print ("\nPress: \n	'x' To disconnect (not implemented yet the reconnection)\n	'+' '-' To increment or decrement speed \n	' ' To take off \n	'enter' To land \n	'Del' To Emergency \n	'i'-'k'-'j'-'l' To move vertically and rotate \n    'w'-'s'-'a'-'d' To move horizontally \n    'g' To go to a waypoint \n    't' To follow a path \n    'm' To change control mode: 0 = Action commands 1 = Velocity commands \n    'v' To send a velocity command \n")

        if s == S_CONNECTED:
            getch2 = _GetchUnix()
            key = getch2()

            # call services
            if key == 'm':
                rospy.loginfo("Srv call: Change command mode")
                print("Introduce:")
                print ("\n 0 If you want to command by Actions")
                print ("\n 1 If you want to command by Velocity commands")
                mode = raw_input()
                print (change_mode_client(int(mode)))

            elif key == 'x':
                rospy.loginfo("Srv call: Disconnect")
                print (disconnect_client())

            elif key == '+':
                rospy.loginfo("Srv call: +")
                print (increase_client())
                if drone_speed < 100:
                    drone_speed += 10

            elif key == '-':
                rospy.loginfo("Srv call: -")
                print (decrease_client())
                if drone_speed > 0:
                    drone_speed -= 10

            elif ord(key) == 127:
                rospy.loginfo("Srv call: Emergency ")
                print (emergency_client())

            # call actions
            elif key == ' ':
                result = take_off_client()
                print ('Drone taken off (z = ' + str(result.final_z) + ')')

            elif ord(key) == 13:
                result = land_client()
                print ('Drone landed (z = ' + str(result.final_z) + ')')

            elif key == 'g':
                print('Introduce x coordinate:')
                x = raw_input()
                print('Introduce y coordinate:')
                y = raw_input()
                print('Introduce z coordinate:')
                z = raw_input()
                print('Introduce yaw:')
                yaw = raw_input()
                result = goto_waypoint_client(float(x), float(y), float(z), float(yaw))
                print ('Waypoint achieved (Pose = [' + str(result.final_pose.position.x) + ', ' + str(result.final_pose.position.y) + ', ' + str(result.final_pose.position.z) + ']  yaw = ' + str(result.final_pose.orientation.z))

            elif key == 't':
                print('Introduce "1" to follow a straight-line path or "0" to follow a circular path:')
                m = raw_input()
                mode = bool(m)
                if mode == 1:
                    print('Introduce W1 and W2 coordinates:')
                    print('W1x:')
                    W1x = raw_input()
                    print('W1y:')
                    W1y = raw_input()
                    print('W2x:')
                    W2x = raw_input()
                    print('W2y:')
                    W2y = raw_input()
                    print('Introduce Velocity:')
                    v = raw_input()
                    w1 = [float(W1x), float(W1y)]
                    w2 = [float(W2x), float(W2y)]
                if mode == 0:
                    print('Introduce Origin coordinates and Radius:')
                    print('Ox:')
                    Ox = raw_input()
                    print('Oy:')
                    Oy = raw_input()
                    print('R:')
                    R = raw_input()
                    print('Introduce Velocity:')
                    v = raw_input()
                    w1 = [float(Ox), float(Oy)]
                    w2 = [float(R), 0]

                result = path_following_client(mode, w1, w2, v)
                print ('Path followed (Pose = [' + str(result.final_pose.position.x) + ', ' + str(
                    result.final_pose.position.y) + ', ' + str(result.final_pose.position.z) + ']  yaw = ' + str(
                    result.final_pose.orientation.z))


            # publish cmd_velocity
            elif key == 'v':
                print('Moving with a speed')
                print('vx:')
                vx = raw_input()
                print('vy:')
                vy = raw_input()
                print('vz:')
                vz = raw_input()
                print('vyaw:')
                vyaw = raw_input()
                velocity_msg.linear.x = float(vx)
                velocity_msg.linear.y = float(vy)
                velocity_msg.linear.z = float(vz)
                velocity_msg.angular.z = float(vyaw)
                rospy.loginfo("Moving drone with v=("+str(velocity_msg.linear.x)+","+str(velocity_msg.linear.y)+","+str(velocity_msg.linear.z)+") linear speed and w="+str(velocity_msg.angular.z)+" angular speed")
                pub_velocity.publish(velocity_msg)

            # publish cmd_incr_vel
            elif key == 'i':
                incr_vel_msg.linear.x = 0
                incr_vel_msg.linear.y = 0
                incr_vel_msg.linear.z = drone_speed
                incr_vel_msg.angular.z = 0
                rospy.loginfo("Move up ")
                pub_incr_vel.publish(incr_vel_msg)

            elif key == 'k':
                incr_vel_msg.linear.x = 0
                incr_vel_msg.linear.y = 0
                incr_vel_msg.linear.z = -drone_speed
                incr_vel_msg.angular.z = 0
                rospy.loginfo("Move down ")
                pub_incr_vel.publish(incr_vel_msg)

            elif key == 'l':
                incr_vel_msg.linear.x = 0
                incr_vel_msg.linear.y = 0
                incr_vel_msg.linear.z = 0
                incr_vel_msg.angular.z = drone_speed
                rospy.loginfo("Turn right ")
                pub_incr_vel.publish(incr_vel_msg)

            elif key == 'j':
                incr_vel_msg.linear.x = 0
                incr_vel_msg.linear.y = 0
                incr_vel_msg.linear.z = 0
                incr_vel_msg.angular.z = -drone_speed
                rospy.loginfo("Turn left")
                pub_incr_vel.publish(incr_vel_msg)

            elif key == 'd':
                incr_vel_msg.linear.x = drone_speed
                incr_vel_msg.linear.y = 0
                incr_vel_msg.linear.z = 0
                incr_vel_msg.angular.z = 0
                rospy.loginfo("Move in x")
                pub_incr_vel.publish(incr_vel_msg)

            elif key == 'a':
                incr_vel_msg.linear.x = -drone_speed
                incr_vel_msg.linear.y = 0
                incr_vel_msg.linear.z = 0
                incr_vel_msg.angular.z = 0
                rospy.loginfo("Move in -x")
                pub_incr_vel.publish(incr_vel_msg)

            elif key == 'w':
                incr_vel_msg.linear.x = 0
                incr_vel_msg.linear.y = -drone_speed
                incr_vel_msg.linear.z = 0
                incr_vel_msg.angular.z = 0
                rospy.loginfo("Move in y")
                pub_incr_vel.publish(incr_vel_msg)

            elif key == 's':
                incr_vel_msg.linear.x = 0
                incr_vel_msg.linear.y = drone_speed
                incr_vel_msg.linear.z = 0
                incr_vel_msg.angular.z = 0
                rospy.loginfo("Move in -y")
                pub_incr_vel.publish(incr_vel_msg)

            rate.sleep()

def cmd_square():
    global state
    wp1 = [0.0, 0.0, 1.25, -1.57]
    wp2 = [0.5, 0.0, 1.25, -1.57]
    wp3 = [0.5, 0.0, 0.75, -1.57]
    wp4 = [0.0, 0.0, 0.75, -1.57]

    print ("CONTROL DRONE MODE: AUTONOMOUS COMMAND WAYPOINTS. Draw a square and land \n")

    result1 = connect_client()
    print ('Drone connected (state = ' + str(result1.result) + ')')

    # despega
    result2 = take_off_client()
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (z = ' + str(result2.final_z) + ')')
    else:
        print ('Drone taken off (z = ' + str(result2.final_z) + ')')

    # dibujar un cuadrado
    time.sleep(1)
    result3 = goto_waypoint_client(wp1[0], wp1[1], wp1[2], wp1[3])
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(result3.final_pose.orientation.z))
    else:
        print ('Waypoint achieved (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(result3.final_pose.orientation.z))
    time.sleep(1)
    result4 = goto_waypoint_client(wp2[0], wp2[1], wp2[2],  wp2[3])
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (Pose = [' + str(result4.final_pose.position.x) + ', ' + str(result4.final_pose.position.y) + ', ' + str(result4.final_pose.position.z) + ']  yaw = ' + str(result4.final_pose.orientation.z))
    else:
        print ('Waypoint achieved (Pose = [' + str(result4.final_pose.position.x) + ', ' + str(result4.final_pose.position.y) + ', ' + str(result4.final_pose.position.z) + ']  yaw = ' + str(result4.final_pose.orientation.z))
    time.sleep(1)
    result5 = goto_waypoint_client(wp3[0], wp3[1], wp3[2],  wp3[3])
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print ('Drone disconnected (Pose = [' + str(result5.final_pose.position.x) + ', ' + str(result5.final_pose.position.y) + ', ' + str(result5.final_pose.position.z) + ']  yaw = ' + str(result5.final_pose.orientation.z))
    else:
        print ('Waypoint achieved (Pose = [' + str(result5.final_pose.position.x) + ', ' + str(result5.final_pose.position.y) + ', ' + str(result5.final_pose.position.z) + ']  yaw = ' + str(result5.final_pose.orientation.z))
    time.sleep(1)
    result6 = goto_waypoint_client(wp4[0], wp4[1], wp4[2], wp4[3])
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print ('Drone disconnected (Pose = [' + str(result6.final_pose.position.x) + ', ' + str(result6.final_pose.position.y) + ', ' + str(result6.final_pose.position.z) + ']  yaw = ' + str(result6.final_pose.orientation.z))
    else:
        print ('Waypoint achieved (Pose = [' + str(result6.final_pose.position.x) + ', ' + str(result6.final_pose.position.y) + ', ' + str(result6.final_pose.position.z) + ']  yaw = ' + str(result6.final_pose.orientation.z))
    time.sleep(1)
    result7 = land_client()
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (z = ' + str(result7.final_z) + ')')
    else:
        print ('Drone landed (z = ' + str(result7.final_z) + ')')

    rospy.spin()

def cmd_gotopoint():
    global state
    wp0 = [2.0, 2.0, 2.0]
    des_yaw = 0.0

    print ("CONTROL DRONE MODE: AUTONOMOUS COMMAND WAYPOINT. Go to a waypoint and exit \n")

    result1 = connect_client()
    print ('Drone connected (state = ' + str(result1.result) + ')')

    result2 = take_off_client()
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (z = ' + str(result2.final_z) + ')')
    else:
        print ('Drone taken off (z = ' + str(result2.final_z) + ')')

    result3 = goto_waypoint_client(wp0[0], wp0[1], wp0[2], des_yaw)
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(result3.final_pose.orientation.z))
    else:
        print ('Waypoint achieved (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(result3.final_pose.orientation.z))

    print ('End of program')
    rospy.spin()


def cmd_stayinpoint():
    global state
    wp0 = [2.0, 2.0, 2.0]
    des_yaw = 0.0

    print ("CONTROL DRONE MODE: AUTONOMOUS COMMAND WAYPOINT. Go to a waypoint and stay \n")

    result1 = connect_client()
    print ('Drone connected (state = ' + str(result1.result) + ')')

    result2 = take_off_client()
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (z = ' + str(result2.final_z) + ')')
    else:
        print ('Drone taken off (z = ' + str(result2.final_z) + ')')
    result3 = stayin_waypoint_client(wp0[0], wp0[1], wp0[2], des_yaw)
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(
            result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(
            result3.final_pose.orientation.z))
    else:
        print ('Waypoint achieved (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(
            result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(
            result3.final_pose.orientation.z))

    rospy.spin()

def cmd_line_trajectory():
    global state
    mode = 1

    w1_x = rospy.get_param('~param_w1_x', 1)
    w1_x = float(w1_x)
    w1_y = rospy.get_param('~param_w1_y', 1)
    w1_y = float(w1_y)
    w2_x = rospy.get_param('~param_w2_x', 2)
    w2_x = float(w2_x)
    w2_y = rospy.get_param('~param_w2_y', 1.5)
    w2_y = float(w2_y)

    w1 = [w1_x, w1_y]
    w2 = [w2_x, w2_y]
    v = 0.7

    v = float(v)

    print ("CONTROL DRONE MODE: AUTONOMOUS FOLLOW LINE TRAJECTORY. \n")

    result1 = connect_client()
    print ('Drone connected (state = ' + str(result1.result) + ')')

    result2 = take_off_client()
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (z = ' + str(result2.final_z) + ')')
    else:
        print ('Drone taken off (z = ' + str(result2.final_z) + ')')
    time.sleep(4)
    result3 = path_following_client(mode, w1, w2, v)
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(
            result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(
            result3.final_pose.orientation.z))
    else:
        print ('Path followed (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(
            result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(
            result3.final_pose.orientation.z))

    rospy.spin()


def cmd_connect():
    global state

    print ("CONTROL DRONE MODE: CONNECT MULTIPLE DRONES. \n")

    result1 = connect_client()
    print ('Drone connected (state = ' + str(result1.result) + ')')

    rospy.spin()



def cmd_loiter_trajectory():
    global state
    mode = 0
    O = [float(0), float(2)]
    r = [float(1), float(0)]
    v = 0.5

    v = float(v)

    print ("CONTROL DRONE MODE: AUTONOMOUS FOLLOW LOITER TRAJECTORY. \n")

    result1 = connect_client()
    print ('Drone connected (state = ' + str(result1.result) + ')')

    result2 = take_off_client()
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (z = ' + str(result2.final_z) + ')')
    else:
        print ('Drone taken off (z = ' + str(result2.final_z) + ')')
    time.sleep(4)
    result3 = path_following_client(mode, O, r, v)
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(
            result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(
            result3.final_pose.orientation.z))
    else:
        print ('Path followed (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(
            result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(
            result3.final_pose.orientation.z))

    rospy.spin()


def cmd_waypoints_trajectory():
    global state
    mode = 2
    # trayectoria con interpolacion cubica de los waypoints
    spline = 3
    wp = [[1,1],[2,1.5],[3,0],[2.5,-1]]
    v = 0.5

    v = float(v)
    # pasar wp a vector
    numrows = len(wp)
    numcols = len(wp[0])
    wp_send = []
    for i in range(0, numrows):
        for j in range(0, numcols):
            wp_send.append(float(wp[i][j]))

    print ("CONTROL DRONE MODE: AUTONOMOUS FOLLOW LOITER TRAJECTORY. \n")

    result1 = connect_client()
    print ('Drone connected (state = ' + str(result1.result) + ')')

    result2 = take_off_client()
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (z = ' + str(result2.final_z) + ')')
    else:
        print ('Drone taken off (z = ' + str(result2.final_z) + ')')
    time.sleep(4)
    result3 = path_following_client(mode, spline, wp_send, v)
    mutex.acquire()
    s = state
    mutex.release()
    if s != S_CONNECTED:
        print('Drone disconnected (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(
            result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(
            result3.final_pose.orientation.z))
    else:
        print ('Path followed (Pose = [' + str(result3.final_pose.position.x) + ', ' + str(
            result3.final_pose.position.y) + ', ' + str(result3.final_pose.position.z) + ']  yaw = ' + str(
            result3.final_pose.orientation.z))

    rospy.spin()



if __name__ == '__main__':
    global drone_speed, state
    drone_speed = 30
    state = 0

    rospy.Subscriber('/drone' + str(num_drone) + '/state', Int16, state_callback)

    try:
        # cmd_gotopoint()
        # cmd_stayinpoint()
        # cmd_keyboard()
        # cmd_square()
        cmd_line_trajectory()
        # cmd_connect()
        # cmd_loiter_trajectory()
        # cmd_waypoints_trajectory()
    except rospy.ROSInterruptException:
        pass
