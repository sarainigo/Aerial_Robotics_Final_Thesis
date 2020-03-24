#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import math


a0 = 0.6
a1 = -1.0
a2 = -1.0

my_w1_x = -1.0
my_w1_y = 0.0
my_w2_x = 0.0
my_w2_y = 1.0

            #vector_info = [0d, 1dist, 2a0, 3a1, 4a2, 5W1[0], 6W1[1], 7W2[0], 8W2[1], 9act_pose[0], 10act_pose[1], 11act_pose[2],12act_yaw, 13act_t, 14cmd_x, 15cmd_y, 17act_vel[0], act_vel[1], act_vel[2], act_vel[3], act_accel[0],act_accel[1], act_accel[2], act_Va, Va_des, Vax_des, Vay_des, cmd_va, cmd_va_drone, act_phi, phi_des, act_phi_x, act_phi_y, phi_des_x, phi_des_y]

matrix_all = np.loadtxt("./data/info_drone_" + str(a0) + "_" + str(a1) + "_" + str(a2) + ".txt")
t = matrix_all[:, 12]
x = matrix_all[:, 8]
y = matrix_all[:, 9]
z = matrix_all[:, 10]
cmd_x = matrix_all[:, 13]
cmd_y = matrix_all[:, 14]
d = matrix_all[:, 0]
vx = matrix_all[:, 15]
vy= matrix_all[:, 16]

dim = t.shape[0]

# recta dada
m = (my_w2_y-my_w1_y)/(my_w2_x-my_w1_x + 1e-6)
n = -m*my_w1_x+my_w1_y
c = [m,n]
pol = np.poly1d(c)
x_des = np.linspace(min(x),max(x),dim)

plt.figure('PATH FOLLOWING CARROT CHASING ALGORITHM')


# DIAGRAMA XY

plt.subplot(221)
plt.plot(x_des, pol(x_des), 'g-', x, y, 'b--', label='a0=' + str(a0)+' a1=' + str(a1)+' a2=' + str(a2))
plt.title('Diagrama XY')
plt.xlabel('x')
plt.ylabel('y')
plt.grid(True)
# legend = plt.legend(loc='best', shadow=True)
legend = plt.legend(loc='upper right', shadow=True)



# DIAGRAMA Z

plt.subplot(222)
plt.plot(t, z, '-', label='a0=' + str(a0)+' a1=' + str(a1)+' a2=' + str(a2))
plt.title('Diagrama Z frente al tiempo')
plt.xlabel('t')
plt.ylabel('z')
plt.grid(True)
# legend = plt.legend(loc='best', shadow=True)
legend = plt.legend(loc='upper right', shadow=True)


# DIAGRAMA DIST

matrix_all = np.loadtxt("./data/info_drone_" + str(a0) + "_" + str(a1) + "_" + str(a2) + ".txt")
d = matrix_all[:, 1]

CrossTrack = 0
for l in range(0, len(d)): 
   CrossTrack = CrossTrack + d[l]

plt.subplot(223)
plt.plot(t, d, '-', label='Cross-track error=' + str(CrossTrack))
plt.title('Cross track')
plt.xlabel('t')
plt.ylabel('Cross-track error')
plt.grid(True)
# legend = plt.legend(loc='best', shadow=True)
legend = plt.legend(loc='upper right', shadow=True)

plt.subplot(224)
plt.plot(t, vx, 'r-',t, vy, 'b-', label='velocity')
plt.title('Vel')
plt.xlabel('t')
plt.ylabel('Velocity')
plt.grid(True)
# legend = plt.legend(loc='best', shadow=True)
legend = plt.legend(loc='upper right', shadow=True)

 
plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25,
                                wspace=0.35)


plt.show()

