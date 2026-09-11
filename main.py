import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

from map import map as Map
from robot import robot
from target import can_pick_up, target

MAP_WIDTH = 10
MAP_HEIGHT = 10
FRAME_DELAY_SECONDS = 0.25

def _robot_vertices(robot):
    """Return a direction-facing triangle centred on the robot."""
    heading = robot.orientation
    direction = np.array([math.cos(heading), math.sin(heading)])
    perpendicular = np.array([-direction[1], direction[0]])
    centre = np.array([robot.x, robot.y])

    # The tip and base remain inside one grid cell and form an isosceles triangle.
    return np.array(
        [
            centre + 0.35 * direction,
            centre - 0.25 * direction + 0.27 * perpendicular,
            centre - 0.25 * direction - 0.27 * perpendicular,
        ]
    )


class MapView:
    """Keep the grid fixed and reuse patches for the changing entities."""

    def __init__(self, world, ax):
        self.ax = ax
        self.canvas = ax.figure.canvas
        self.shape = (world.width, world.height)
        self.robot_patches = {}
        self.target_patches = {}
        self.background = None
        self.use_blit = self.canvas.supports_blit

        ax.set_facecolor("white")
        ax.set_xticks(np.arange(world.width))
        ax.set_yticks(np.arange(world.height))
        ax.set_xticks(np.arange(-0.5, world.width, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, world.height, 1), minor=True)
        ax.grid(which="minor", color="#cbd5e1", linestyle="-", linewidth=0.8)
        ax.tick_params(which="minor", bottom=False, left=False)
        ax.set_xlim(-0.5, world.width - 0.5)
        ax.set_ylim(-0.5, world.height - 0.5)
        ax.set_aspect("equal")
        ax.set_title(f"{world.width}x{world.height} 2D Grid")
        ax.figure.tight_layout()
        self.draw_callback = self.canvas.mpl_connect("draw_event", self._on_draw)

    def _draw_entities(self):
        # Targets sit underneath robots, including when they share a cell.
        for patch in (*self.target_patches.values(), *self.robot_patches.values()):
            self.ax.draw_artist(patch)

    def _on_draw(self, event):
        # A resize, pan or zoom rebuilds the static background automatically.
        if self.use_blit and not self.canvas.is_saving():
            self.background = self.canvas.copy_from_bbox(self.ax.bbox)
            self._draw_entities()
        else:
            self.background = None

    def update(self, world):
        robots = set(world.robots.values())
        targets = {
            item for row in world.blocks for block in row for item in block.targets
        }
        for patches, entities in (
            (self.robot_patches, robots), (self.target_patches, targets)
        ):
            for entity in list(patches):
                if entity not in entities:
                    patches.pop(entity).remove()

        for item in targets:
            if item not in self.target_patches:
                patch = Rectangle(
                    (0, 0), 0.55, 0.55, facecolor="green", edgecolor="darkgreen",
                    animated=self.use_blit, zorder=2,
                )
                self.target_patches[item] = self.ax.add_patch(patch)
            self.target_patches[item].set_xy((item.x - 0.275, item.y - 0.275))

        for item in robots:
            vertices = _robot_vertices(item)
            if item not in self.robot_patches:
                patch = Polygon(
                    vertices, closed=True, facecolor="#FF0000",
                    animated=self.use_blit, zorder=3,
                )
                self.robot_patches[item] = self.ax.add_patch(patch)
            else:
                self.robot_patches[item].set_xy(vertices)

    def render(self):
        if not self.use_blit or self.background is None:
            self.canvas.draw()
        else:
            self.canvas.restore_region(self.background)
            self._draw_entities()
            self.canvas.blit(self.ax.bbox)
        self.canvas.flush_events()


def draw_map(world, ax=None):
    """Update a persistent map view; pass the returned axes for later frames."""
    if ax is None:
        fig, ax = plt.subplots(figsize=(6, 6))
    else:
        fig = ax.figure
    view = getattr(ax, "_map_view", None)
    if view is None or view.shape != (world.width, world.height):
        if view is not None:
            fig.canvas.mpl_disconnect(view.draw_callback)
        ax.clear()
        view = ax._map_view = MapView(world, ax)
    view.update(world)
    view.render()
    return fig, ax


def _show_frame(world, ax=None):
    """Render one simulation frame in the same window as the previous frame."""
    fig, ax = draw_map(world, ax=ax)
    # pause() requests a full redraw for stale artists, defeating blitting.
    fig.canvas.start_event_loop(FRAME_DELAY_SECONDS)
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

def add_random_robots(world, count):
    """Add a number of robots at random positions and orientations."""
    for _ in range(count):
        x = np.random.randint(0, world.width)
        y = np.random.randint(0, world.height)
        orientations = [0, 90, 180, 270]
        orientation = np.random.choice(orientations)
        world.add_robot(robot(x, y, orientation))

def add_random_targets(world, count):
    """Add a number of targets at random positions."""
    for _ in range(count):
        x = np.random.randint(0, world.width)
        y = np.random.randint(0, world.height)
        world.add_target(target(x, y))

def test_check_pickups(case):
    """Test pickup results and state cleanup at the end of an iteration."""
    world = Map(10, 10)
    if case == 1:
        world.add_robot(robot(3, 4, 0))
        current_robot = next(iter(world.robots.values()))

        plt.ion()
        _, ax = _show_frame(world)
        current_robot.forward()
        check_pickups(world)
        _show_frame(world,ax=ax)
        current_robot.forward()
        check_pickups(world)
        _show_frame(world, ax=ax)
        current_robot.pickUp()
        check_pickups(world)
        _show_frame(world, ax=ax)
        plt.ioff()
        plt.show()
    if case == 2:
        world.add_robot(robot(3, 4, 0))
        world.add_target(target(5, 4))
        current_robot = next(iter(world.robots.values()))

        plt.ion()
        _, ax = _show_frame(world)
        current_robot.forward()
        check_pickups(world)
        _show_frame(world,ax=ax)
        current_robot.forward()
        check_pickups(world)
        _show_frame(world, ax=ax)
        current_robot.pickUp()
        check_pickups(world)
        _show_frame(world, ax=ax)
        plt.ioff()
        plt.show()
    if case == 3:
        world.add_robot(robot(3, 4, 0))
        world.add_robot(robot(7, 4, 180))
        world.add_target(target(5, 4))
        robot1 = list(world.robots.values())[0]
        robot2 = list(world.robots.values())[1]

        plt.ion()
        _, ax = _show_frame(world)
        robot1.forward()
        robot2.forward()
        check_pickups(world)
        _show_frame(world, ax=ax)
        robot1.forward()
        robot2.forward()
        check_pickups(world)
        _show_frame(world, ax=ax)
        robot1.pickUp()
        robot2.pickUp()
        check_pickups(world)
        _show_frame(world, ax=ax)
        plt.ioff()
        plt.show()
    if case == 4:
        world.add_robot(robot(3, 4, 0))
        world.add_robot(robot(7, 4, 180))
        world.add_robot(robot(5, 2, 90))
        world.add_target(target(5, 4))
        robot1 = list(world.robots.values())[0]
        robot2 = list(world.robots.values())[1]
        robot3 = list(world.robots.values())[2]

        plt.ion()
        _, ax = _show_frame(world)
        robot1.forward()
        robot2.forward()
        robot3.forward()
        check_pickups(world)
        _show_frame(world, ax=ax)
        robot1.forward()
        robot2.forward()
        robot3.forward()
        check_pickups(world)
        _show_frame(world, ax=ax)
        robot1.pickUp()
        robot2.pickUp()
        robot3.pickUp()
        check_pickups(world)
        _show_frame(world, ax=ax)
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    test_check_pickups(3)

    # world = Map(MAP_WIDTH, MAP_HEIGHT)
    # Add robots to `world` with `world.add_robot(robot(...))` before drawing.
    # world.add_robot(robot(0, 0, 270))

    # for row in world.blocks:
    #     for block in row:
    #         for current_robot in block.robots:
    #             current_robot.forward()
    #             current_robot._sense(world)

    # draw_map(world)
    # plt.show()
