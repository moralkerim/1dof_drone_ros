import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

# =====================
# Fiziksel parametreler
# =====================
m = 1.0        # drone kütlesi (kg)
m1 = 0.5       # motor1 kütkesi (kg)
m2 = 0.5       # motor2 kütlesi (kg)
L = 0.5        # motorlar arası mesafe / 2 (m)
I = 0.02       # drone atalet momenti (kg*m^2)
g = 9.81       # yerçekimi (m/s^2)
dt = 0.01      # zaman adımı

# Başlangıç durumu
theta = 0.0    # açı (radyan)
omega = 0.0    # açısal hız (rad/s)

# Motor kuvvetleri (N)
F1 = 5.0
F2 = 5.0

# =====================
# Simülasyon fonksiyonu
# =====================
def update_state(theta, omega, F1, F2):
    # Toplam tork
    torque = (F2 - F1 + m1*g - m2*g) * L * math.cos(theta)
    alpha = torque / I  # açısal ivme
    omega += alpha * dt
    theta += omega * dt
    return theta, omega

# =====================
# Animasyon hazırlığı
# =====================
fig, ax = plt.subplots()
ax.set_xlim(-1, 1)
ax.set_ylim(-0.5, 0.5)
ax.set_aspect('equal')
line, = ax.plot([], [], 'o-', lw=4)

def init():
    line.set_data([], [])
    return line,

def animate(frame):
    global theta, omega, F1, F2
    theta, omega = update_state(theta, omega, F1, F2)
    
    # Drone'u çiz
    x = np.array([-L, L])
    y = np.array([0, 0])
    x_rot = x * np.cos(theta) - y * np.sin(theta)
    y_rot = x * np.sin(theta) + y * np.cos(theta)
    line.set_data(x_rot, y_rot)
    return line,

ani = FuncAnimation(fig, animate, frames=1000, interval=10, blit=True, init_func=init)
plt.show()
