import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
# Define grid size
grid_size = (5, 5)

# Define agents with start and goal positions
agents = [
    {"start": (0, 0), "goal": (4, 4), "path": [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]},
    {"start": (0, 4), "goal": (4, 0), "path": [(0, 4), (1, 3), (2, 2), (3, 1), (4, 0)]}
]

# Create figure and axis
fig, ax = plt.subplots()
ax.set_xlim(0, grid_size[0])
ax.set_ylim(0, grid_size[1])

# Plot grid lines
for i in range(grid_size[0] + 1):
    ax.axhline(i, color='gray', linestyle='--')
for j in range(grid_size[1] + 1):
    ax.axvline(j, color='gray', linestyle='--')

# Initialize plot elements
lines = [ax.plot([], [], 'o-')[0] for _ in range(len(agents))]

# Update function for animation
def update(frame):
    for i, agent in enumerate(agents):
        if frame < len(agent["path"]):
            x, y = agent["path"][frame]
        else:
            x, y = agent["path"][-1]
        lines[i].set_data(x, y)
    return lines


# Create animation
anim = FuncAnimation(fig, update, frames=range(max(len(agent["path"]) for agent in agents)), interval=500, blit=True)

# Save frames to WSL
for i in range(max(len(agent["path"]) for agent in agents)):
    plt.savefig(f'frame_{i}.png')

plt.show()

plt.close()  # 关闭图形窗口
