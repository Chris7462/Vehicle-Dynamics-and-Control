import numpy as np
import matplotlib.pyplot as plt

# Given / assumed "known info" (parameters + initial state)
L  = 2.7        # wheelbase [m]
v  = 10.0       # constant speed [m/s]
dt = 0.1        # time step [s] (10 Hz update rate)
Ld = 6.0        # lookahead distance [m]

x_path = [8, 16, 24, 32, 40, 46, 50, 52, 50, 45, 38, 30, 24, 22, 24, 30, 40, 52, 64, 74]
y_path = [0,  0,  0,  2,  6, 12, 20, 28, 36, 42, 46, 48, 46, 40, 34, 30, 28, 30, 36, 46]

# Simulation length (simple cap)
T = 30.0
N = int(T / dt)
t = np.linspace(0.0, T, N + 1)

# Define the variables we are calculating, set them to zero to initialize them
x_veh = np.zeros(N + 1)
y_veh = np.zeros(N + 1)
theta = np.zeros(N + 1)
delta = np.zeros(N + 1)   # store steering if you want to plot later (optional)
last_idx = 0


# Helper functions
def wrap_to_pi(angle_rad):
    """Wrap angle to [-pi, pi]."""
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


def dist(x1, y1, x2, y2):
    """Euclidean distance between two points."""
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def select_path_point(x, y, x_path, y_path, Ld, last_idx):
    """Select lookahead waypoint by searching forward from last_idx
    for the first waypoint with distance >= Ld.
    Also advances last_idx past waypoints the vehicle has passed."""
    # Search forward for first waypoint >= Ld
    best_i = None
    for i in range(last_idx, len(x_path)):
        d = dist(x, y, x_path[i], y_path[i])
        if d >= Ld:
            best_i = i
            break

    # Fallback: use final waypoint
    if best_i is None:
        best_i = len(x_path) - 1

    # Advance last_idx past waypoints already passed
    new_last = last_idx
    for i in range(last_idx, best_i):
        if dist(x, y, x_path[i], y_path[i]) < Ld:
            new_last = i + 1

    x_path_point = x_path[best_i]
    y_path_point = y_path[best_i]
    return x_path_point, y_path_point, new_last


def ppc_steering_angle_calc(x_veh, y_veh, theta, x_path_point, y_path_point, l, Ld):
    """Compute PPC steering angle."""
    alpha = np.arctan2(y_path_point - y_veh, x_path_point - x_veh) - theta
    alpha = wrap_to_pi(alpha)
    delta = np.arctan(2 * l * np.sin(alpha) / Ld)
    return delta


def veh_xy_update_bicycle_model(x_veh, y_veh, theta, v, L, delta, dt):
    """Forward Euler update using bicycle model."""
    x_veh_new = x_veh + v * np.cos(theta) * dt
    y_veh_new = y_veh + v * np.sin(theta) * dt
    theta_new = theta + (v / L) * np.tan(delta) * dt
    return x_veh_new, y_veh_new, theta_new


# Run discrete simulation
for k in range(N):

    # (1) Select target point
    x_pp, y_pp, last_idx = select_path_point(
        x_veh[k], y_veh[k], x_path, y_path, Ld, last_idx)

    # (2) Pure pursuit steering
    Ld_actual = dist(x_veh[k], y_veh[k], x_pp, y_pp)
    delta[k] = ppc_steering_angle_calc(
        x_veh[k], y_veh[k], theta[k], x_pp, y_pp, L, Ld_actual)

    # (3) Move the vehicle using the Bicycle model
    x_veh[k+1], y_veh[k+1], theta[k+1] = veh_xy_update_bicycle_model(
        x_veh[k], y_veh[k], theta[k], v, L, delta[k], dt)

    # Optional: wrap heading for readability
    theta[k + 1] = wrap_to_pi(theta[k + 1])

    # Simple stopping condition (no more waypoints or close to final waypoint?)
    if dist(x_veh[k+1], y_veh[k+1], x_path[-1], y_path[-1]) < 2.0:
        break

# Trim arrays to actual simulation length
x_veh = x_veh[:k+2]
y_veh = y_veh[:k+2]
theta = theta[:k+2]
delta = delta[:k+2]

# ----------------------------
# Plot (path vs vehicle)
# ----------------------------
plt.figure()
plt.plot(x_path, y_path, 'o', label='Waypoints')
plt.plot(x_path, y_path, '-', label='Path')
plt.plot(x_veh, y_veh, '-', linewidth=2, label='Vehicle')

plt.axis('equal')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.grid(True)
plt.legend()
plt.show()
