import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

from map import map as Map
from robot import robot
from target import can_pick_up, target

MAP_WIDTH = 10
MAP_HEIGHT = 10
FRAME_DELAY_SECONDS = 0.05

ITERATIONS = 200
RANDOM_ROBOTS = 2
RANDOM_TARGETS = 1

def _robot_vertices(robot):
    """Return a direction-facing triangle centred on the robot."""
    direction = np.array(robot.DIRECTIONS[robot.orientation], dtype=float)
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
    if not isinstance(view, MapView) or view.shape != (world.width, world.height):
        if isinstance(view, MapView):
            fig.canvas.mpl_disconnect(view.draw_callback)
        ax.clear()
        view = MapView(world, ax)
        setattr(ax, "_map_view", view)
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


def move_towards_goal(current_robot):
    """Use one forward/turn action to move the robot towards its goal."""
    delta_x = current_robot.goal_x - current_robot.x
    delta_y = current_robot.goal_y - current_robot.y

    if delta_x == 0 and delta_y == 0:
        return

    # Finish the horizontal part of the route first, then the vertical part.
    if delta_x > 0:
        desired_orientation = 0  # East
    elif delta_x < 0:
        desired_orientation = 2  # West
    elif delta_y > 0:
        desired_orientation = 1  # North
    else:
        desired_orientation = 3  # South

    if current_robot.orientation == desired_orientation:
        current_robot.forward()
        return

    left_turns = (desired_orientation - current_robot.orientation) % 4
    right_turns = (current_robot.orientation - desired_orientation) % 4
    if left_turns <= right_turns:
        current_robot.turnLeft()
    else:
        current_robot.turnRight()

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
    try:
        world = Map(MAP_WIDTH, MAP_HEIGHT)

        plt.ion()
        _, ax = _show_frame(world)
        # Add robots to `world`
        add_random_robots(world, RANDOM_ROBOTS)
        # Add targets to `world`
        add_random_targets(world, RANDOM_TARGETS)

        for i in range(ITERATIONS):
            for robot in world.robots.values():
                # Robot communication handling (receiving messages)
                payload = robot.receive()
                if robot.state == "searching":
                    if payload is not None and payload["message"] == "Target found":
                        if robot.state != "moving_to_target":
                            robot.goal_x, robot.goal_y = payload["position"]
                            robot.state = "moving_to_target"
                if robot.state == "found_target":
                    if payload is not None and payload["message"] == "Moving to target" and payload["goal"] == (robot.x, robot.y):
                        robot.state = "waiting_for_pickup"

            for robot in world.robots.values():
                robot.sense(world)
                # Robot action decision-making
                if robot.state == "searching":
                    if robot.foundTargetAhead:
                        robot.forward()
                    elif robot.foundTargetCurrent:
                        robot.state = "found_target"
                    else:
                        move_direction = np.random.choice(
                            ["forward", "left", "right"],
                            p=[0.6, 0.2, 0.2],
                        )
                        if move_direction == "forward":
                            robot.forward()
                        elif move_direction == "left":
                            robot.turnLeft()
                        elif move_direction == "right":
                            robot.turnRight()
                elif robot.state == 'moving_to_target':
                    if (robot.x, robot.y) == (robot.goal_x, robot.goal_y):
                        robot.state = "waiting_for_pickup"
                    else:
                        move_towards_goal(robot)

            check_pickups(world) # Check for pickups after all robots have moved
            for robot in world.robots.values():
                # Robot communication handling (sending messages)
                if robot.state == "found_target":
                    robot.send("Target found")
                if robot.state == "moving_to_target":
                    robot.send("Moving to target", goal_x=robot.goal_x, goal_y=robot.goal_y)


                # Update the map view after each robot's action
                _show_frame(world,ax=ax)
    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
