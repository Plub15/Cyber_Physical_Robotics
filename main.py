import argparse
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

from map import map as Map
from message_debugger import MessageDebugger, MessageTrace
from robot import robot
from robot_logging import configure_robot_logs, close_robot_logs, open_log_viewers, log_robot_changes
from rules import check_illegal_behavior
from target import can_pick_up, target

MAP_WIDTH = 10
MAP_HEIGHT = 10
FRAME_DELAY_SECONDS = 1

ITERATIONS = 20000
RANDOM_ROBOTS = 4
RANDOM_TARGETS = 1

PROMISE_THRESHOLD = 2  # Number of promises required for a robot to consider another robot as a helper

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
        self.robot_labels = {}
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
        self.draw_callback = self.canvas.mpl_connect("draw_event", self._on_draw)

    def _draw_entities(self):
        # Targets sit underneath robots, including when they share a cell.
        for patch in (*self.target_patches.values(), *self.robot_patches.values(), *self.robot_labels.values()):
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
            (self.robot_patches, robots), (self.target_patches, targets),
            (self.robot_labels, robots),
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
            if item not in self.robot_labels:
                self.robot_labels[item] = self.ax.text(
                    item.x, item.y + 0.36, f"R{item.id}", ha="center", va="bottom",
                    fontsize=8, animated=self.use_blit, zorder=4,
                )
            self.robot_labels[item].set_position((item.x, item.y + 0.36))

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
    if ax is fig.axes[0] and len(fig.axes) == 1 and view.background is None:
        fig.tight_layout()
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


def move_to_goal(current_robot):
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


def run_iteration(world, iteration):
    """Receive, sense/decide, validate, act, then send for one iteration."""

    for current_robot in world.robots.values():
        current_robot.iteration = iteration

    for robot in world.robots.values():
        # Robot communication handling (receiving messages)
        for payload in robot.inbox[:]:
            if robot.receive(payload) is None:
                continue
            if robot.state == "searching":
                if payload["message"] == "Need_help":
                    robot.state = "volunteer"
                    robot.task_id = payload["task_id"]
                    robot.goal_x, robot.goal_y = payload["goal"]
            elif robot.state == "needs_help":
                if payload["message"] == "Help_offer" and payload.get("task_id") == robot.task_id:
                    if payload["distance_to_goal"] < robot.helper_robot_distance:
                        robot.helper_robot_id = payload["from"]
                        robot.helper_robot_distance = payload.get("distance_to_goal")
                robot.patience_counter -= 1
                if robot.patience_counter <= 0:
                    if robot.helper_robot_id is not None:
                        robot.state = "wait_for_helper"
                        robot.patience_counter = 10 # Reset
                    else:
                        robot.state = "searching"

            #Paxos
            # Proposer; Only "Found Target" robot
            elif robot.state == "PREPARED":
                if robot.promise < 0.5*(RANDOM_ROBOTS): #Less than majority of robot
                    if payload["message"] == "PROMISE" and payload["ResponseID"] == robot.id and payload.get("ballot") == robot.proposal_ballot:
                        robot.promise += 1
                        if payload["prevCommand"] is not None:
                            robot.commandHeard = payload["prevCommand"]
                    robot.patience_counter -= 1
                else:
                    robot.state = "ACCEPT_REQUEST"
                    robot.accepter = 0
                if robot.patience_counter <= 0:
                    # Try again with higher message_value if no helper robot is found within the patience limit.
                    robot.state = "wait_for_helper"
                    robot.patience_counter = 10 # Reset
                    robot.message_value = max(robot.message_value, robot.highestMessageValueHeard)

            elif robot.state == "ACCEPT_REQUESTED":
                if robot.accepter < 0.5*(RANDOM_ROBOTS): #Less than majority of robot
                    if payload["message"] == "ACCEPT" and payload["ResponseID"] == robot.id and payload.get("ballot") == robot.proposal_ballot:
                        robot.accepter += 1
                    robot.patience_counter -= 1
                else:
                    robot.state = "COMMANDER"

                if robot.patience_counter <= 0:
                    # Try again with higher message_value if no helper robot is found within the patience limit.
                    robot.state = "wait_for_helper"
                    robot.patience_counter = 10 # Reset
                    robot.message_value = max(robot.message_value, robot.highestMessageValueHeard)

            elif robot.state == "wait_for_assignment":
                if payload["message"] == "EXECUTE":
                    if robot.id == robot.command[0]:
                        robot.goal_x, robot.goal_y = robot.command[1], robot.command[2]
                        robot.state = "move_to_goal"
                    else:
                        #Release
                        robot.state = "searching"
                        robot.goal_x, robot.goal_y = None, None
                        robot.command = set()
                        robot.prevCommand = None
                        robot.ResponseID = None
                        robot.commandHeard = None


            # Incoming promises never change this robot's outgoing proposal ballot.
            incoming_ballot = payload.get("ballot")
            if incoming_ballot is not None:
                robot.highestMessageValueHeard = max(
                    robot.highestMessageValueHeard, incoming_ballot[0])
            if payload["message"] in ("PREPARE", "ACCEPT_REQUEST"):
                if incoming_ballot is None or incoming_ballot < robot.promised_ballot:
                    continue
                robot.promised_ballot = incoming_ballot
                robot.response_ballot = incoming_ballot
                robot.ResponseID = payload["from"]
                if payload["message"] == "PREPARE":
                    robot.response = "PROMISE"
                else:
                    robot.response = "ACCEPT"
                    robot.command = payload["Command"]

            log_robot_changes(robot, "receive")
        log_robot_changes(robot, "communication")

    planned_actions = {}
    for current_robot in world.robots.values():
        # Sense the environment and decide on an action based on the robot's state and surroundings.
        current_robot.sense(world)
        log_robot_changes(current_robot, "sense")
        action = None
        if current_robot.state == "searching":
            if current_robot.foundTargetAhead and not current_robot.foundRobotAhead:
                action = "forward"
                current_robot.state = "found_target"
            elif current_robot.foundTargetAhead and current_robot.foundRobotAhead:
                action = np.random.choice(["left", "right"], p=[0.5, 0.5])
            else:
                action = np.random.choice(["forward", "left", "right"], p=[0.6, 0.2, 0.2])

        elif current_robot.state == "found_target":
            if current_robot.NumfoundRobotCurrent == 2:
                action = "pick_up"
                robot.state = "searching"
            elif current_robot.NumfoundRobotCurrent == 1:
                current_robot.goal_x, current_robot.goal_y = current_robot.x, current_robot.y
                current_robot.state = "needs_help"
                current_robot.patience_counter = 10 # Reset patience counter when entering needs_help state
                current_robot.task_id = f"task_{iteration}_{current_robot.id}"
            elif current_robot.NumfoundRobotCurrent > 2:
                if current_robot.id in current_robot.RobotIDsInCurrentBlock[:2]:
                    action = "pick_up"
                    robot.state = "searching"
                else:
                    action = np.random.choice(["forward", "left", "right"], p=[0.6, 0.2, 0.2])
                    robot.state = "searching"

        elif current_robot.state == "move_to_goal":
            if current_robot.x == current_robot.goal_x and current_robot.y == current_robot.goal_y:
                action = "pick_up"
                #Release
                current_robot.state = "searching"
                current_robot.goal_x, current_robot.goal_y = None, None
                current_robot.command = set()
                current_robot.prevCommand = None
                current_robot.ResponseID = None
                current_robot.commandHeard = None
            else:
                action = move_to_goal(current_robot)

        elif current_robot.state == "wait_for_pickup":
            if current_robot.helper_robot_id in current_robot.RobotIDsInCurrentBlock:
                action = "pick_up"
                #Release
                current_robot.state = "searching"
                current_robot.goal_x, current_robot.goal_y = None, None
                current_robot.command = set()
                current_robot.prevCommand = None
                current_robot.ResponseID = None
                current_robot.commandHeard = None

        log_robot_changes(current_robot, "decision")
        planned_actions[current_robot] = action

    approved_actions = check_illegal_behavior(planned_actions)
    perform_actions(world, approved_actions)

    for robot in world.robots.values():
        # Robot communication handling (sending messages)
        if robot.response == None:
            if robot.state == "needs_help":
                robot.send("Need_help", goal_x=robot.x, goal_y=robot.y, task_id=robot.task_id)
            elif robot.state == "volunteer":
                robot.send("Help_offer", goal_x=robot.goal_x, goal_y=robot.goal_y, task_id=robot.task_id)
                robot.state = "wait_for_assignment"
            elif robot.state == "wait_for_helper":
                robot.message_value = max(robot.message_value, robot.highestMessageValueHeard, robot.promised_ballot[0]) + 1
                robot.promise = 0
                robot.proposal_ballot = (robot.message_value, robot.id)
                robot.send("PREPARE", ballot=robot.proposal_ballot)
                robot.state = "PREPARED"
            elif robot.state == "ACCEPT_REQUEST":
                if robot.commandHeard is not None:
                    command = robot.commandHeard
                else:
                    command = (robot.helper_robot_id, robot.x, robot.y)
                robot.command = command
                robot.send("ACCEPT_REQUEST", goal_x=robot.goal_x, goal_y=robot.goal_y, task_id=robot.task_id, Command=command, ballot=robot.proposal_ballot)
                robot.state = "ACCEPT_REQUESTED"
            elif robot.state == "COMMANDER":
                robot.send("EXECUTE")
                if robot.commandHeard is not None:
                    robot.state = "wait_for_helper" # Retry again for it own command
                else:
                    robot.state = "wait_for_pickup" # Wait for pickup

        # Acceptor; All robot is acceptor
        else:
            if robot.response == "PROMISE":
                robot.send("PROMISE", prevCommand=robot.prevCommand, ResponseID=robot.ResponseID, ballot=robot.response_ballot)
            elif robot.response == "ACCEPT":
                robot.send("ACCEPT", ResponseID=robot.ResponseID, ballot=robot.response_ballot)
                robot.prevCommand = robot.command

            robot.response = None
        log_robot_changes(robot, "send")


def main():
    parser = argparse.ArgumentParser(description="Run the robot simulation.")
    parser.add_argument("--debug-panes", action="store_true", help="Open a live log pane for each robot.")
    parser.add_argument("--no-debugger", action="store_true", help="Show only the map, without the message debugger.")
    parser.add_argument("--iterations", type=int, default=ITERATIONS, help="Number of simulation iterations.")
    parser.add_argument("--seed", type=int, help="Random seed for repeatable movement and message delays.")
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error("--iterations must be positive")
    if args.seed is not None:
        np.random.seed(args.seed)
    world = None
    try:
        world = Map(MAP_WIDTH, MAP_HEIGHT)

        plt.ion()
        # Add robots to `world`
        # add_random_robots(world, RANDOM_ROBOTS)
        robot1 = robot(2, 2, 0)  # Robot at (2, 2) facing East
        robot2 = robot(2, 7, 0)  # Robot at (2, 7) facing East
        robot3 = robot(7, 2, 0)  # Robot at (7, 2) facing East
        target1 = target(8, 2)  # Target at (8, 2)
        robot_for_test = [robot1, robot2, robot3]
        for r in robot_for_test:
            world.add_robot(r)
        world.add_target(target1)

        log_directory = configure_robot_logs(world.robots.values())
        print(f"Robot logs: {log_directory}")
        if args.debug_panes:
            open_log_viewers(log_directory)
        debugger = None
        if args.no_debugger:
            fig, ax = _show_frame(world)
        else:
            trace = MessageTrace()
            trace.attach(world.robots.values())
            debugger = MessageDebugger(world, trace, draw_map, FRAME_DELAY_SECONDS)
            debugger.update(0)
            fig = debugger.fig
            plt.show(block=False)

        i = 0
        last_step = time.monotonic() - FRAME_DELAY_SECONDS
        while plt.fignum_exists(fig.number):
            if debugger is not None:
                fig.canvas.start_event_loop(0.05)
                if debugger.closed:
                    break
                if not debugger.steps and time.monotonic() - last_step < debugger.interval:
                    continue
                if not debugger.ready():
                    continue
            elif i >= args.iterations:
                break
            if not any(block.targets for row in world.blocks for block in row):
                add_random_targets(world, RANDOM_TARGETS)
            run_iteration(world, i + 1)
            # Render after every robot has performed its action and sent messages.
            i += 1
            last_step = time.monotonic()
            if debugger is not None:
                debugger.finished = i >= args.iterations
                debugger.update(i)
            else:
                _show_frame(world, ax=ax)

            print(
                f"\r[Iteration: {i}/{args.iterations}] [Score: {world.score}]",
                end="", flush=True,
            )
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    finally:
        if world is not None:
            print(f"\nFinal score: {world.score}")
            close_robot_logs(world.robots.values())
        plt.close("all")


if __name__ == "__main__":
    main()
