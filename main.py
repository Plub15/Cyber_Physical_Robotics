import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

from map import map as Map
from robot import robot
from robot_logging import configure_robot_logs, close_robot_logs, open_log_viewers, log_robot_changes
from rules import check_illegal_behavior
from target import can_pick_up, target

MAP_WIDTH = 10
MAP_HEIGHT = 10
FRAME_DELAY_SECONDS = 0.03

ITERATIONS = 20000
RANDOM_ROBOTS = 16
RANDOM_TARGETS = 50

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
        for participant in current_block.robots:
            if participant.logger is not None:
                participant.logger.info("PICKUP position=%s success=%s readyToPick=%s",
                                        block_position, picked_up, participant.readyToPick)

        if picked_up:
            world.remove_target(current_block.targets[0])
            world.score += 1

    # Prepare for the next iteration.
    for robot in world.robots.values():
        robot.readyToPick = False
        log_robot_changes(robot, "pickup")

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


def decide_goal_action(current_robot):
    """Choose one forward/turn action towards the goal without executing it."""
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
        return "forward"

    left_turns = (desired_orientation - current_robot.orientation) % 4
    right_turns = (current_robot.orientation - desired_orientation) % 4
    if left_turns <= right_turns:
        return "left"
    return "right"


def decide_action(current_robot):
    """Update decision state and choose an action without changing the world."""
    action = None
    if current_robot.state == "waiting_for_pickup":
        action = "pick_up"
        current_robot.state = "searching"
        current_robot.helper_robot_id = None
        current_robot.helper_robot_distance = float("inf")

    elif current_robot.state == "give_up":
        action = np.random.choice(["forward", "left", "right"], p=[0.6, 0.2, 0.2])
        current_robot.state = "searching"
        current_robot.helper_robot_id = None
        current_robot.helper_robot_distance = float("inf")
        current_robot.patience_counter = 20

    elif current_robot.state == "searching":
        if current_robot.foundTargetAhead and not current_robot.foundRobotAhead:
            action = "forward"
        elif current_robot.foundTargetAhead and current_robot.foundRobotAhead:
            action = np.random.choice(["left", "right"], p=[0.5, 0.5])
        elif current_robot.foundTargetCurrent and current_robot.NumfoundRobotCurrent == 1:
            current_robot.state = "found_target"
            current_robot.patience_counter = 20
        else:
            action = np.random.choice(["forward", "left", "right"], p=[0.6, 0.2, 0.2])

    elif current_robot.state == "moving_to_target":
        if (current_robot.x, current_robot.y) == (current_robot.goal_x, current_robot.goal_y):
            current_robot.state = "target_reached"
        else:
            action = decide_goal_action(current_robot)

    if current_robot.logger is not None:
        current_robot.logger.debug("DECISION action=%s", action)
    log_robot_changes(current_robot, "decision")
    return action


def perform_actions(world, approved_actions):
    """Execute a validated plan, collecting each approved target only once."""
    picked_targets = {}
    for current_robot, action in approved_actions.items():
        if action == "forward":
            current_robot.forward(validated=True)
        elif action == "left":
            current_robot.turnLeft()
        elif action == "right":
            current_robot.turnRight()
        elif action == "pick_up":
            current_robot.pickUp()
            current_block = world.get_block(current_robot.x, current_robot.y)
            picked_targets[(current_block.x, current_block.y)] = current_block.targets[0]
        log_robot_changes(current_robot, "action")

    # Both members of a pickup pair act before its target is removed. Legality
    # has already been checked, so execution does not re-evaluate the attempts.
    for item in picked_targets.values():
        world.remove_target(item)
        world.score += 1
    for current_robot in world.robots.values():
        current_robot.readyToPick = False
        log_robot_changes(current_robot, "pickup")

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


def run_iteration(world, iteration, target_count=RANDOM_TARGETS):
    """Receive, sense/decide, validate, act, then send for one iteration."""
    if not any(block.targets for row in world.blocks for block in row):
        add_random_targets(world, target_count)

    for current_robot in world.robots.values():
        current_robot.iteration = iteration

    for robot in world.robots.values():
        # Robot communication handling (receiving messages)
        for payload in robot.inbox[:]:
            if robot.receive(payload) is None:
                continue
            if robot.state == "searching":
                if payload is not None and payload["message"] == "Target found":
                    robot.goal_x, robot.goal_y = payload["position"]
                    robot.state = "available_to_help"
                    robot.patience_counter = 20

            if robot.state == "found_target":
                if (
                    payload is not None
                    and payload["message"] == "available_to_help"
                    and payload["goal"] == (robot.x, robot.y)
                ):
                    if payload.get("distance_to_goal") < robot.helper_robot_distance:
                        robot.helper_robot_id = payload["from"]
                        robot.helper_robot_distance = payload.get(
                            "distance_to_goal", float("inf")
                        )

            if robot.state == "available_to_help":
                if (
                    payload is not None
                    and payload["message"] == "waiting_for_help"
                    and payload.get("goal") == (robot.goal_x, robot.goal_y)
                    and payload.get("helper_robot_id") == robot.id
                ):
                    robot.state = "moving_to_target"
                elif (
                    payload is not None
                    and payload["message"] == "waiting_for_help"
                    and payload.get("goal") == (robot.goal_x, robot.goal_y)
                    and payload.get("helper_robot_id") != robot.id
                ):
                    robot.state = "searching"


            if robot.state == "waiting_for_help":
                if (
                    payload is not None
                    and payload["message"] == "Target reached"
                    and payload.get("goal") == (robot.x, robot.y)
                ):
                    robot.state = "waiting_for_pickup"

            log_robot_changes(robot, "receive")

        if robot.state == "found_target" and robot.helper_robot_id is not None:
            robot.state = "waiting_for_help"

        if robot.state in ["found_target","available_to_help"]:
            robot.patience_counter -= 1
            if robot.patience_counter <= 0:
                robot.state = "give_up"
        log_robot_changes(robot, "communication")

    planned_actions = {}
    for current_robot in world.robots.values():
        current_robot.sense(world)
        log_robot_changes(current_robot, "sense")
        planned_actions[current_robot] = decide_action(current_robot)

    approved_actions = check_illegal_behavior(planned_actions)
    perform_actions(world, approved_actions)

    for robot in world.robots.values():
        # Robot communication handling (sending messages)
        if robot.state == "found_target":
            robot.send("Target found", goal_x=robot.x, goal_y=robot.y)
        if robot.state == "moving_to_target":
            robot.send("Moving to target", goal_x=robot.goal_x, goal_y=robot.goal_y)
        if robot.state == "target_reached":
            robot.send("Target reached", goal_x=robot.goal_x, goal_y=robot.goal_y)
            robot.state = "waiting_for_pickup"
        if robot.state == "waiting_for_help":
            robot.send("waiting_for_help", goal_x=robot.x, goal_y=robot.y)
        if robot.state == "available_to_help":
            robot.send("available_to_help", goal_x=robot.goal_x, goal_y=robot.goal_y)
        log_robot_changes(robot, "send")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the robot simulation.")
    parser.add_argument("--debug-panes", action="store_true", help="Open a live log pane for each robot.")
    args = parser.parse_args()
    world = None
    try:
        world = Map(MAP_WIDTH, MAP_HEIGHT)

        plt.ion()
        _, ax = _show_frame(world)
        # Add robots to `world`
        add_random_robots(world, RANDOM_ROBOTS)
        log_directory = configure_robot_logs(world.robots.values())
        print(f"Robot logs: {log_directory}")
        if args.debug_panes:
            open_log_viewers(log_directory)
        for i in range(ITERATIONS):
            run_iteration(world, i + 1)
            # Render after every robot has performed its action and sent messages.
            _show_frame(world,ax=ax)

            print(
                f"\r[Iteration: {i + 1}/{ITERATIONS}] [Score: {world.score}]",
                end="", flush=True,
            )
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    finally:
        if world is not None:
            print(f"\nFinal score: {world.score}")
            close_robot_logs(world.robots.values())
