#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


def convertir_matriz(lista, n):
    return list(zip(*[iter(lista)] * n))


def talker():
    pub = rospy.Publisher('mytopic', Float32MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    wp_send_vector = Float32MultiArray()
    wp = [[1,1],[2,1.5],[3,0],[2.5,-1]]
    numrows = len(wp)  # 3 rows in your example
    numcols = len(wp[0])
    print(numrows)
    print(numcols)
    wp_send_vector = Float32MultiArray()
    wp_send = []
    wp = [[1,1],[2,1.5],[3,0],[2.5,-1]]

    for i in range(0, numrows):
        for j in range(0, numcols):
            wp_send.append(float(wp[i][j]))

    wp_send_vector.data = wp_send
    rospy.loginfo(wp_send_vector)
    pub.publish(wp_send_vector)

    matriz=zip(wp_send[::2], wp_send[1::2])
    print(len(matriz))
    print(matriz)

    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass