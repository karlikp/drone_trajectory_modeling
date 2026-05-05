import numpy as np


class Drone3DOF:
    """
    Simplified drone motion model in 3 DOF:
    state: x = [px, py, pz, vx, vy, vz]
    input: u = [T, phi, theta, F_dist]
    """

    def __init__(self, mass=2.5, kx=0.15, ky=0.15, kz=0.25, g=9.81):
        self.mass = mass
        self.kx = kx
        self.ky = ky
        self.kz = kz
        self.g = g

    def derivatives(self, t, state, control_function):
        px, py, pz, vx, vy, vz = state

        T, phi, theta, F_dist, mass = control_function(t)

        dxdt = vx
        dydt = vy
        dzdt = vz

        dvxdt = (T * theta - self.kx * vx) / mass
        dvydt = (T * phi - self.ky * vy) / mass
        dvzdt = (T - mass * self.g - self.kz * vz - F_dist) / mass

        return np.array([
            dxdt,
            dydt,
            dzdt,
            dvxdt,
            dvydt,
            dvzdt
        ])