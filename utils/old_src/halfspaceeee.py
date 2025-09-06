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
bounding_box = Polygon([[-3, -3], [3, -3], [3, 3], [-3, 3]])
obstacle_circle = Point(center[0], center[1]).buffer(rho)
safe_region = bounding_box.difference(obstacle_circle)
# new target
y = np.array([-1.0, 2.0])      # user‑requested

# projection onto half‑space H(z)
d = np.dot(y - mid, normal)
if d <= 0:          # already inside
    proj = y.copy()
    moved = False
else:
    proj = y - d * normal
    moved = True

# --------------- plotting ----------------------------
fig, ax = plt.subplots(figsize=(6,6))
ax.grid(False)

# shade half‑space
xx, yy = np.meshgrid(np.linspace(-2.5, 3, 400),
                     np.linspace(-2,   3, 400))
mask = np.dot(np.stack([xx.ravel(), yy.ravel()], axis=1) - mid, normal) <= 0
ax.contourf(xx, yy, mask.reshape(xx.shape),
            levels=[-0.1, 0.1], colors=['#e5fbe5'], alpha=0.99)

# draw obstacle
th = np.linspace(0, 2*np.pi, 400)
ax.plot(center[0] + rho*np.cos(th), center[1] + rho*np.sin(th),
        'k', lw=2)
x_obs, y_obs = obstacle_circle.exterior.xy
ax.fill(x_obs, y_obs, color='gray', alpha=0.8, label='Obstacle')
ax.text(-0, -0.35,'Obstacle',fontsize=12,color='w')
# z and o(z)
ax.plot(z[0], z[1], 'ro', label='$z$')
ax.text(z[0] + 0.1, z[1] + 0.1, 'current state $z$', fontsize=14, color='b')
ax.plot(o[0], o[1], 'bo', label=r'$o_i(z)$')
ax.text(o[0], o[1] + 0.15, '$o_i(z)$', fontsize=14, color='b')

ax.plot([z[0], o[0]], [z[1], o[1]], 'r--', lw=1)
ax.text(o[0]+0.3 , o[1] - 0.25, '$H_i(z)$', fontsize=14, color='g')


# # half‑space boundary
line_pts = mid[:, None] + np.array([normal, -normal]).T * np.linspace(-4,4,2)
ax.plot(line_pts[0], line_pts[1], 'g', lw=2, label='boundary')

# target and projection
ax.plot(y[0], y[1], marker='*', color='gold', markersize=13, label='$y$')
ax.text(y[0] + 0.1, y[1] + 0.3, 'target $y$', fontsize=14, color='b')
if moved:
    ax.plot(proj[0], proj[1], 'ms', markersize=7, label=r'$\pi_z(y)$')
    ax.text(proj[0]+0.1,proj[1]-0.2,'$\pi_z(y)$',fontsize=14, color='b')
    ax.plot([y[0], proj[0]], [y[1], proj[1]], 'm--', lw=1)

# nav vector
ax.arrow(z[0], z[1], proj[0]-z[0], proj[1]-z[1],
         head_width=0.08, length_includes_head=True,
         color='purple', lw=2, label=r'$n_{\mathrm{sph}}$')
ax.text(1.52,1.46,'This is $n_{\mathrm{sph}}$!!',fontsize=22, color='purple')

ax.set_aspect('equal')
ax.set_xlim(-2.5, 3)
ax.set_ylim(-2, 3)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title(r'Computing NavSphere')
# ax.legend(loc='upper left', fontsize=8)

plt.show()
