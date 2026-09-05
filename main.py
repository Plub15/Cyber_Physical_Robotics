import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

from map import map as Map
from robot import robot
from target import can_pick_up, target

MAP_WIDTH = 10
MAP_HEIGHT = 10

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


def check_pickups(world):
    """Resolve each pickup location once, removing successfully picked targets."""
    checked_blocks = set()
    results = []

    for robot in world.robots.values():
        if not robot.readyToPick:
            continue

        current_block = world.get_block(robot.x, robot.y)
        block_position = (current_block.x, current_block.y)

        # Check each pickup location only once per iteration.
        if block_position in checked_blocks:
            continue

        checked_blocks.add(block_position)
        picked_up = can_pick_up(robot)
        results.append((block_position, picked_up))

        if picked_up:
            world.remove_target(current_block.targets[0])

    # Prepare for the next iteration.
    for robot in world.robots.values():
        robot.readyToPick = False

    return results


def test_check_pickups():
    """Test pickup results and state cleanup at the end of an iteration."""
    cases = [
        ("no target", 1, False, False),
        ("one robot", 1, True, False),
        ("exactly two robots", 2, True, True),
        ("more than two robots", 3, True, False),
    ]

    for name, robot_count, has_target, expected in cases:
        world = Map(3, 3)

        for _ in range(robot_count):
            bot = robot(1, 1, 0)
            bot.readyToPick = True
            world.add_robot(bot)

        if has_target:
            world.add_target(target(1, 1))

        results = check_pickups(world)
        assert results == [((1, 1), expected)]
        remaining_targets = len(world.get_block(1, 1).targets)
        assert remaining_targets == (0 if expected else int(has_target))
        assert all(not bot.readyToPick for bot in world.robots.values())
        print(f"check_pickups test ({name}): PASS")


if __name__ == "__main__":
    test_check_pickups()

    world = Map(MAP_WIDTH, MAP_HEIGHT)
    # Add robots to `world` with `world.add_robot(robot(...))` before drawing.
    # world.add_robot(robot(0, 0, 270))

    # for row in world.blocks:
    #     for block in row:
    #         for current_robot in block.robots:
    #             current_robot.forward()
    #             current_robot._sense(world)

    # draw_map(world)
    # plt.show()
