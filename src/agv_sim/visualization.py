"""Matplotlib visualization."""

import matplotlib.pyplot as plt


def plot_result(grid, path, start, goal, title="AGV Static Obstacle Avoidance"):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.imshow(grid.occupancy, origin="lower", cmap="Greys")
    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, linewidth=2, label="A* path")
    ax.scatter([start[0]], [start[1]], s=70, label="Start")
    ax.scatter([goal[0]], [goal[1]], s=70, label="Goal")
    ax.set_xlabel("X [grid cell]")
    ax.set_ylabel("Y [grid cell]")
    ax.set_title(title)
    ax.legend()
    ax.grid(alpha=0.15)
    fig.tight_layout()
    return fig
