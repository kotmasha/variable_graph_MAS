import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
# Geometry parameters (same obstacle + z as before)
center = np.array([0.0, 0.0])
rho = 1.0
z = np.array([1.8, 0.3])

direction = z - center
o = center + rho * direction / np.linalg.norm(direction)

mid = 0.5 * (z + o)
normal = o - z
normal /= np.linalg.norm(normal)

# new target
y = np.array([-1.0, 2.0])      # user‑requested
bounding_box = Polygon([[-3, -3], [3, -3], [3, 3], [-3, 3]])
obstacle_circle = Point(center[0], center[1]).buffer(rho)
# Compute the difference: workspace minus obstacle
safe_region = bounding_box.difference(obstacle_circle)
# projection onto half‑space H(z)
d = np.dot(y - mid, normal)
if d <= 0:          # already inside
    proj = y.copy()
    moved = False
else:
    proj = y - d * normal
    moved = True

# Start plot
fig, ax = plt.subplots(figsize=(6, 6))
ax.grid(False)
# --------------- plotting ----------------------------
fig, ax = plt.subplots(figsize=(6,6))
ax.grid(False)
# Plot safe region (light blue)
x_safe, y_safe = safe_region.exterior.xy
ax.fill(x_safe, y_safe, color='lightblue', alpha=0.5, label='Safe workspace')

# Plot obstacle region (gray fill)
x_obs, y_obs = obstacle_circle.exterior.xy
ax.fill(x_obs, y_obs, color='gray', alpha=0.8, label='Obstacle')
# shade half‑space
xx, yy = np.meshgrid(np.linspace(-2.5, 3, 400),
                     np.linspace(-2,   3, 400))
mask = np.dot(np.stack([xx.ravel(), yy.ravel()], axis=1) - mid, normal) <= 0
ax.contourf(xx, yy, mask.reshape(xx.shape),
            levels=[-0.1, 0.1], colors=['#e5fbe5'], alpha=0.99)
theta = np.linspace(0, 2*np.pi, 400)
ax.plot(center[0] + rho*np.cos(theta), center[1] + rho*np.sin(theta), 'k', lw=2)

# z and o(z)
ax.plot(z[0], z[1], 'ro', label='$z$')
ax.plot(o[0], o[1], 'bo', label=r'$o_i(z)$')
ax.plot([z[0], o[0]], [z[1], o[1]], 'r--', lw=1)

# half‑space boundary
line_pts = mid[:, None] + np.array([normal, -normal]).T * np.linspace(-4,4,2)
ax.plot(line_pts[0], line_pts[1], 'g', lw=2, label='boundary')

# target and projection
ax.plot(y[0], y[1], marker='x', color='red', markersize=13, label='$y$')
if moved:
    ax.plot(proj[0], proj[1], 'ms', markersize=7, label=r'$\pi_z(y)$')
    ax.plot([y[0], proj[0]], [y[1], proj[1]], 'm--', lw=1)
# nav vector
ax.arrow(z[0], z[1], proj[0]-z[0], proj[1]-z[1],
         head_width=0.08, length_includes_head=True,
         color='purple', lw=2, label=r'$n_{\mathrm{sph}}$')

plt.show()