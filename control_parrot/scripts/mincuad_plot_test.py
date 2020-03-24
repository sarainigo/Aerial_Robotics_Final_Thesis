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
plt.subplot(331)
plt.plot(t1, x1, '.', tlin_x1, polx1(tlin_x1), '-')
plt.title('Interpolacion de x PRUEBA 1')
plt.xlabel('t')
plt.ylabel('x')
plt.grid(True)

plt.subplot(332)
plt.plot(t2, x2, '.', tlin_x2, polx2(tlin_x2), '-')
plt.title('Interpolacion de x PRUEBA 2')
plt.xlabel('t')
plt.ylabel('x')
plt.grid(True)

plt.subplot(333)
plt.plot(t3, x3, '.', tlin_x3, polx3(tlin_x3), '-')
plt.title('Interpolacion de x PRUEBA 3')
plt.xlabel('t')
plt.ylabel('x')
plt.grid(True)


polvx1_nuevo = np.polyder(polx1)

plt.subplot(334)
plt.plot(tlin_x1, polvx1_nuevo(tlin_x1), '-')
plt.title('Interpolacion de v PRUEBA 1')
plt.xlabel('t')
plt.ylabel('vx')
plt.ylim(min(polvx1_nuevo(tlin_x1))-0.1,max(polvx1_nuevo(tlin_x1))+0.1)
plt.grid(True)

polvx2_nuevo = np.polyder(polx2)

plt.subplot(335)
plt.plot(tlin_x2, polvx2_nuevo(tlin_x2), '-')
plt.title('Interpolacion de v PRUEBA 2')
plt.xlabel('t')
plt.ylabel('vx')
plt.ylim(min(polvx2_nuevo(tlin_x2))-0.1,max(polvx2_nuevo(tlin_x2))+0.1)
plt.grid(True)

polvx3_nuevo = np.polyder(polx3)

plt.subplot(336)
plt.plot(tlin_x3, polvx3_nuevo(tlin_x3), '-')
plt.title('Interpolacion de v PRUEBA 3')
plt.xlabel('t')
plt.ylabel('vx')
plt.ylim(min(polvx3_nuevo(tlin_x3))-0.1,max(polvx3_nuevo(tlin_x3))+0.1)
plt.grid(True)



polax1 = np.polyder(polvx1_nuevo)

plt.subplot(337)
plt.plot(tlin_x1, polax1(tlin_x1), '-')
plt.title('Interpolacion de a PRUEBA 1')
plt.xlabel('t')
plt.ylabel('ax')
plt.ylim(min(polax1(tlin_x1))-0.1,max(polax1(tlin_x1))+0.1)
plt.grid(True)

polax2 = np.polyder(polvx2_nuevo)

plt.subplot(338)
plt.plot(tlin_x2, polax1(tlin_x2), '-')
plt.title('Interpolacion de a PRUEBA 2')
plt.xlabel('t')
plt.ylabel('ax')
plt.ylim(min(polax1(tlin_x2))-0.1,max(polax1(tlin_x2))+0.1)
plt.grid(True)

polax3 = np.polyder(polvx3_nuevo)

plt.subplot(339)
plt.plot(tlin_x3, polax1(tlin_x3), '-')
plt.title('Interpolacion de a PRUEBA 3')
plt.xlabel('t')
plt.ylabel('ax')
plt.ylim(min(polax1(tlin_x3))-0.1,max(polax1(tlin_x3))+0.1)
plt.grid(True)



plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25,
                        wspace=0.35)

plt.show()

