import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from Drone3DOF import Drone3DOF


# parametry modelu
drone = Drone3DOF(
    mass=2.5,
    kx=0.15, # k - wspolczynnik oporu dla konkretnej osi
    ky=0.15,
    kz=0.25,
    g=9.81
)

def control_function(t):
    """
    Function defining system inputs (control + disturbances)
    """

    base_mass = 2.5
    water_mass = 0.5

    # Mass increases after water sampling
    if t < 5.0:
        mass = base_mass
    else:
        mass = base_mass + water_mass

    # Thrust computed for initial mass
    T = base_mass * drone.g

    # pitch angle - motion along x axis [1–3 s]
    if 1.0 <= t <= 3.0:
        theta = np.deg2rad(5.0)
    else:
        theta = 0.0

    # roll angle - motion along y axis [3–4 s]
    if 3.0 <= t <= 4.0:
        phi = np.deg2rad(5.0)
    else:
        phi = 0.0

    # vertical disturbance (contact with water)
    if 6.0 <= t <= 6.5:
        F_dist = 2.0
    else:
        F_dist = 0.0

    return T, phi, theta, F_dist, mass


# initial conditions
# state = [x, y, z, vx, vy, vz]
initial_state = np.array([
    0.0,    # x [m]
    0.0,    # y [m]
    1.5,    # z [m]
    0.0,    # vx [m/s]
    0.0,    # vy [m/s]
    0.0     # vz [m/s]
])


# simulation time
t_start = 0.0
t_end = 10.0
t_eval = np.linspace(t_start, t_end, 1000)



# calkowanie rownan ruchu drona
#solve_ivp - function for solving ordinary differential equations (ODE)
solution = solve_ivp(
    # fun: function describing system dynamics
    # takes (t, x) and returns dx/dt
    # i.e. derivatives of the state vector
    fun=lambda t, x: drone.derivatives(t, x, control_function),

  # t_span: simulation time interval (t_start, t_end)
    t_span=(t_start, t_end),

    # y0: initial state of the system
    # state vector at t = t_start
    # [x, y, z, vx, vy, vz]
    y0=initial_state,

    # t_eval: time points at which the solution is stored
    t_eval=t_eval,

    # method: numerical integration method
    # RK45 = Runge-Kutta 4/5 order
    method="RK45"
)


# extract results
t = solution.t
x = solution.y[0]
y = solution.y[1]
z = solution.y[2]
vx = solution.y[3]
vy = solution.y[4]
vz = solution.y[5]


# 3D trajectory plot
fig = plt.figure(figsize=(9, 6))
ax = fig.add_subplot(111, projection="3d")

ax.plot(x, y, z, label="Drone flight trajectory")

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
ax.set_title("Drone trajectory in 3DOF model")
ax.legend()
ax.grid(True)

plt.savefig("trajectory_3d.png", dpi=300)


# state position plots
plt.figure(figsize=(10, 6))
plt.plot(t, x, label="x [m]")
plt.plot(t, y, label="y [m]")
plt.plot(t, z, label="z [m]")
plt.axvline(5.0, linestyle="--", label="water sampling")
plt.axvline(6.0, linestyle=":", label="tube disturbance")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.title("Drone position in time function")
plt.legend()
plt.grid(True)
plt.savefig("position_time.png", dpi=300)


# velocity plots
plt.figure(figsize=(10, 6))
plt.plot(t, vx, label="vx [m/s]")
plt.plot(t, vy, label="vy [m/s]")
plt.plot(t, vz, label="vz [m/s]")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.title("Drone velocity in time function")
plt.legend()
plt.grid(True)
plt.savefig("velocity_time.png", dpi=300)


# phase portrait for z axis
plt.figure(figsize=(8, 6))
plt.plot(z, vz)
plt.xlabel("z [m]")
plt.ylabel("vz [m/s]")
plt.title("Phase portrait: altitude z vs vertical velocity vz")
plt.grid(True)
plt.savefig("phase_portrait_z.png", dpi=300)


print("Simulation completed")
print("Generated files:")
print("- trajectory_3d.png")
print("- position_time.png")
print("- velocity_time.png")
print("- phase_portrait_z.png")