#!/usr/bin/env python

# prueba minimos cuadrados para un instante REAL de mi drone.
import matplotlib.pyplot as plt
import numpy as np
import math


x1 = np.loadtxt("plot_mmc_x1.txt", dtype=float)
x2 = np.loadtxt("plot_mmc_x2.txt", dtype=float)
x3 = np.loadtxt("plot_mmc_x3.txt", dtype=float)

t1 = np.loadtxt("plot_mmc_t1.txt", dtype=float)
t2 = np.loadtxt("plot_mmc_t2.txt", dtype=float)
t3 = np.loadtxt("plot_mmc_t3.txt", dtype=float)



n = 30

cx1 = np.polyfit(t1, x1, 3)
# pol
polx1 = np.poly1d(cx1)
tlin_x1 = np.linspace(min(t1), max(t1), 100)

cx2 = np.polyfit(t2, x2, 3)
# pol
polx2 = np.poly1d(cx2)
tlin_x2 = np.linspace(min(t2), max(t2), 100)

cx3 = np.polyfit(t3, x3, 3)
# pol
polx3 = np.poly1d(cx3)
tlin_x3 = np.linspace(min(t3), max(t3), 100)




plt.figure(1)
plt.subplot(311)
plt.plot(t1, x1, 'o', tlin_x1, polx1(tlin_x1), '-')
plt.title('Posicion en x')
plt.xlabel('t')
plt.ylabel('x')
plt.grid(True)



polvx1_nuevo = np.polyder(polx1)

plt.subplot(312)
plt.plot(tlin_x1, polvx1_nuevo(tlin_x1), 'g-')
plt.title('Velocidad en x')
plt.xlabel('t')
plt.ylabel('vx')
plt.ylim(min(polvx1_nuevo(tlin_x1))-0.1,max(polvx1_nuevo(tlin_x1))+0.1)
plt.grid(True)


polax1 = np.polyder(polvx1_nuevo)

plt.subplot(313)
plt.plot(tlin_x1, polax1(tlin_x1), 'g-')
plt.title('Aceleracion en x')
plt.xlabel('t')
plt.ylabel('ax')
plt.ylim(min(polax1(tlin_x1))-0.1,max(polax1(tlin_x1))+0.1)
plt.grid(True)


plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25,
                        wspace=0.35)

plt.show()

