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
    Funkcja odpowiada wymuszeniu układu.
    """

    base_mass = 2.5
    water_mass = 0.5

    # Masa wzrasta po pobraniu próbki wody
    if t < 5.0:
        mass = base_mass
    else:
        mass = base_mass + water_mass

    # Ciąg obliczony dla początkowej masy
    # Po wzroście masy będzie za mały, więc dron zacznie opadać
    T = base_mass * drone.g

    # wychylenie pitch - ruch w osi x [1-3s]
    if 1.0 <= t <= 3.0:
        theta = np.deg2rad(5.0)
    else:
        theta = 0.0

    # wychylenie roll - ruch w osi y [3-4s]
    if 3.0 <= t <= 4.0:
        phi = np.deg2rad(5.0)
    else:
        phi = 0.0

    # zaklocenie pionowe (kontkat z woda)
    if 6.0 <= t <= 6.5:
        F_dist = 2.0
    else:
        F_dist = 0.0

    return T, phi, theta, F_dist, mass


# warunki poczatkowe
# state = [x, y, z, vx, vy, vz]
initial_state = np.array([
    0.0,    # x [m]
    0.0,    # y [m]
    1.5,    # z [m]
    0.0,    # vx [m/s]
    0.0,    # vy [m/s]
    0.0     # vz [m/s]
])


# czas symulacji
t_start = 0.0
t_end = 10.0
t_eval = np.linspace(t_start, t_end, 1000)



# całkowanie równań ruchu drona
# solve_ivp - funkcja do numerycznego rozwiązywania równań różniczkowych (ODE)
solution = solve_ivp(
    # fun: funkcja opisująca układ dynamiczny
    # przyjmuje (t, x) i zwraca dx/dt
    # czyli pochodne wektora stanu
    fun=lambda t, x: drone.derivatives(t, x, control_function),

    # t_span: przedział czasu symulacji (t_start, t_end)
    # solver będzie liczył rozwiązanie od t_start do t_end
    t_span=(t_start, t_end),

    # y0: warunki początkowe układu
    # wektor stanu w chwili t = t_start
    # [x, y, z, vx, vy, vz]
    y0=initial_state,

    # t_eval: punkty czasowe, w których chcemy zapisać rozwiązanie
    # solver może liczyć z adaptacyjnym krokiem,
    # ale wynik zostanie zwrócony dokładnie w tych punktach
    t_eval=t_eval,

    # method: metoda numeryczna całkowania
    # RK45 = Runge-Kutta 4/5 rzędu
    method="RK45"
)


# odczyt wynikow
t = solution.t
x = solution.y[0]
y = solution.y[1]
z = solution.y[2]
vx = solution.y[3]
vy = solution.y[4]
vz = solution.y[5]


# wykres trajektorii 3D
fig = plt.figure(figsize=(9, 6))
ax = fig.add_subplot(111, projection="3d")

ax.plot(x, y, z, label="Trajektoria lotu drona")

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
ax.set_title("Trajektoria lotu drona w modelu 3DOF")
ax.legend()
ax.grid(True)

plt.savefig("trajektoria_3d.png", dpi=300)


# wykresy zmiennych stanu
plt.figure(figsize=(10, 6))
plt.plot(t, x, label="x [m]")
plt.plot(t, y, label="y [m]")
plt.plot(t, z, label="z [m]")
plt.axvline(5.0, linestyle="--", label="pobranie próbki")
plt.axvline(6.0, linestyle=":", label="zakłócenie od rurki")
plt.xlabel("Czas [s]")
plt.ylabel("Położenie [m]")
plt.title("Położenie drona w funkcji czasu")
plt.legend()
plt.grid(True)
plt.savefig("polozenie_czas.png", dpi=300)


# wykres predkosci
plt.figure(figsize=(10, 6))
plt.plot(t, vx, label="vx [m/s]")
plt.plot(t, vy, label="vy [m/s]")
plt.plot(t, vz, label="vz [m/s]")
plt.xlabel("Czas [s]")
plt.ylabel("Prędkość [m/s]")
plt.title("Prędkości drona w funkcji czasu")
plt.legend()
plt.grid(True)
plt.savefig("predkosci_czas.png", dpi=300)


# portret fazowy dla osi z
plt.figure(figsize=(8, 6))
plt.plot(z, vz)
plt.xlabel("z [m]")
plt.ylabel("vz [m/s]")
plt.title("Portret fazowy: wysokość z - prędkość vz")
plt.grid(True)
plt.savefig("portret_fazowy_z.png", dpi=300)


print("Symulacja zakończona.")
print("Wygenerowano pliki:")
print("- trajektoria_3d.png")
print("- polozenie_czas.png")
print("- predkosci_czas.png")
print("- portret_fazowy_z.png")