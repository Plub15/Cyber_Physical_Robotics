import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

from map import map as Map
from robot import robot
from target import target



def _draw_robot(ax, robot):
    """Draw one robot as a red, direction-facing isosceles triangle."""
    heading = robot.orientation
    direction = np.array([math.cos(heading), math.sin(heading)])
    perpendicular = np.array([-direction[1], direction[0]])
    centre = np.array([robot.x, robot.y])

    # The tip and base remain inside one grid cell and form an isosceles triangle.
    vertices = np.array(
        [
            centre + 0.35 * direction,
            centre - 0.25 * direction + 0.27 * perpendicular,
            centre - 0.25 * direction - 0.27 * perpendicular,
        ]
    )
    ax.add_patch(Polygon(vertices, closed=True, facecolor="red", edgecolor="darkred"))


def _draw_target(ax, target):
    """Draw one target as a green square centred in its grid cell."""
    size = 0.55
    ax.add_patch(
        Rectangle(
            (target.x - size / 2, target.y - size / 2),
            size,
            size,
            facecolor="green",
            edgecolor="darkgreen",
        )
    )


def draw_map(world):
    """Display a map and all robots currently registered on it."""
    grid = np.ones((world.height, world.width))
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(grid, cmap="gray", vmin=0, vmax=1, origin="lower")

    ax.set_xticks(np.arange(world.width))
    ax.set_yticks(np.arange(world.height))
    ax.set_xticks(np.arange(-0.5, world.width, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, world.height, 1), minor=True)
    ax.grid(which="minor", color="black", linestyle="-", linewidth=1)
    ax.tick_params(which="minor", bottom=False, left=False)

    for row in world.blocks:
        for block in row:
            for robot in block.robots:
                _draw_robot(ax, robot)
            for target in block.targets:
                _draw_target(ax, target)

    ax.set_xlim(-0.5, world.width - 0.5)
    ax.set_ylim(-0.5, world.height - 0.5)
    ax.set_aspect("equal")
    ax.set_title(f"{world.width}x{world.height} 2D Grid")
    fig.tight_layout()
    return fig, ax


if __name__ == "__main__":
    world = Map(10, 10)
    # Add robots to `world` with `world.add_robot(robot(...))` before drawing.
    world.add_robot(robot(3, 3, 90))
    world.add_robot(robot(2, 3, 0))
    world.add_target(target(3, 4))

    for row in world.blocks:
        for block in row:
            for robot in block.robots:
                robot._sense(world)
    draw_map(world)
    plt.show()

    plt.pause(2)  # Pause for 1 second before updating the map

    for row in world.blocks:
        for block in row:
            for robot in block.robots:
                robot.forward()
    draw_map(world)
    plt.show()
