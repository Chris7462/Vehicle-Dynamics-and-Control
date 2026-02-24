import numpy as np
import matplotlib.pyplot as plt

# Given / assumed "known info" (parameters + initial state)
L       = 2.7       # wheelbase [m] (Nissan Leaf)
v       = 5.0       # constant speed [m/s]
Ld      = 6.0       # lookahead distance [m]
dt      = 0.03      # time step [s] (~33 Hz, matches ROS node timer)

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

# Helper functions
def wrap_to_pi(angle_rad):
    return (angle_rad + np.pi) % (2*np.pi) - np.pi

def dist(x1, y1, x2, y2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def select_path_point(x, y, x_path, y_path, Ld, last_idx=0):
    """Search forward from last_idx for the first waypoint with distance >= Ld.
       Returns (x_point, y_point, updated_last_idx).
    """
    for i in range(last_idx, len(x_path)):
        d = dist(x, y, x_path[i], y_path[i])
        if d >= Ld:
            return x_path[i], y_path[i], i

    # Default: last waypoint
    return x_path[-1], y_path[-1], len(x_path) - 1

def ppc_steering_angle_calc(x_veh, y_veh, theta, x_path_point, y_path_point, L, Ld):
    """Compute PPC steering angle delta from Lec 5 equations."""
    # alpha = angle to lookahead point minus vehicle heading
    alpha = np.arctan2(y_path_point - y_veh, x_path_point - x_veh) - theta
    alpha = wrap_to_pi(alpha)

    # Actual (effective) Ld = true distance to the selected waypoint
    Ld_eff = dist(x_veh, y_veh, x_path_point, y_path_point)
    if Ld_eff < 0.01:          # safety: avoid division by zero
        Ld_eff = Ld

    # PPC steering law:  delta = arctan(2 * L * sin(alpha) / Ld)
    delta = np.arctan2(2.0 * L * np.sin(alpha), Ld_eff)
    return delta


def veh_xy_update_bicycle_model(x_veh, y_veh, theta, v, L, delta, dt):
    """Forward-Euler bicycle model update (same as Lec 4)."""
    x_veh_new   = x_veh + v * np.cos(theta) * dt
    y_veh_new   = y_veh + v * np.sin(theta) * dt
    theta_new   = theta + (v / L) * np.tan(delta) * dt
    return x_veh_new, y_veh_new, theta_new


# Run discrete simulation
last_idx = 0
for k in range(N):

    # (1) Select target point
    xp, yp, last_idx = select_path_point(x_veh[k], y_veh[k], x_path, y_path, Ld, last_idx)
    print(f"xp[{k}] = {xp}, yp[{k}] = {yp}")

    # (2) Pure pursuit steering
    delta[k] = ppc_steering_angle_calc(x_veh[k], y_veh[k], theta[k], xp, yp, L, Ld)
    print(f"delta[{k}] = {delta[k]}")

    # (3) Move the vehicle using the Bicycle model
    x_veh[k+1], y_veh[k+1], theta[k+1] = veh_xy_update_bicycle_model(
        x_veh[k], y_veh[k], theta[k], v, L, delta[k], dt)

    # Optional: wrap heading for readability
    theta[k + 1] = wrap_to_pi(theta[k + 1])
    print(f"x_veh[{k+1}] = {x_veh[k+1]:.3f}, y_veh[{k+1}] = {y_veh[k+1]:.3f}, theta[{k+1}] = {theta[k+1]:.3f}")

    # Simple stopping condition (close to final waypoint?)
    if dist(x_veh[k+1], y_veh[k+1], x_path[-1], y_path[-1]) < 2.0:
        print(f"Reached final waypoint at t = {t[k+1]:.2f} s  (step {k+1})")
        break

# ----------------------------
# Plot (path vs vehicle)
# ----------------------------
plt.figure()
plt.plot(x_path, y_path, 'o', label='Waypoints')
plt.plot(x_path, y_path, '-', label='Path')
plt.plot(x_veh[:k+2], y_veh[:k+2], '-', linewidth=2, label='Vehicle')

plt.axis('equal')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Pure Pursuit Controller — Bicycle Model Simulation")
plt.grid(True)
plt.legend()
plt.show()
