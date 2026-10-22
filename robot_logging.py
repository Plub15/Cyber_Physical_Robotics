"""Per-run robot logs and optional live Windows terminal viewers."""

import base64
from datetime import datetime
import logging
from pathlib import Path
import shutil
import subprocess


class RobotIterationFilter(logging.Filter):
    """Attach the robot's current simulation iteration to every log entry."""

    def __init__(self, robot):
        super().__init__()
        self.robot = robot

    def filter(self, record):
        record.iteration = self.robot.iteration
        return True


def log_robot_changes(robot, phase):
    """Record changed decision inputs and states, including silent transitions."""
    if robot.logger is None:
        return
    snapshot = {
        "state": robot.state,
        "position": (robot.x, robot.y),
        "orientation": robot.orientation,
        "goal": (robot.goal_x, robot.goal_y),
        "distance_to_goal_value": robot.distance_to_goal_value,
        "patience_counter": robot.patience_counter,
        "helper_robot_id": robot.helper_robot_id,
        "helper_robot_distance": robot.helper_robot_distance,
        "readyToPick": robot.readyToPick,
        "foundTargetAhead": robot.foundTargetAhead,
        "foundTargetCurrent": robot.foundTargetCurrent,
        "foundRobotAhead": robot.foundRobotAhead,
        "NumfoundRobotCurrent": robot.NumfoundRobotCurrent,
        "message_value": robot.message_value,
    }
    previous = getattr(robot.logger, "_robot_snapshot", None)
    changes = []
    for name, value in snapshot.items():
        if previous is None:
            changes.append(f"{name}={value!r}")
        elif previous[name] != value:
            changes.append(f"{name}={previous[name]!r}->{value!r}")
    if changes:
        robot.logger.debug("UPDATE phase=%s %s", phase, " ".join(changes))
    robot.logger._robot_snapshot = snapshot


def configure_robot_logs(robots, log_root=None):
    root = Path(log_root) if log_root is not None else Path(__file__).parent / "logs"
    run_directory = root / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run_directory.mkdir(parents=True, exist_ok=False)
    for item in robots:
        logger = logging.Logger(f"Robot {item.id}", level=logging.DEBUG)
        handler = logging.FileHandler(run_directory / f"robot_{item.id}.log", encoding="utf-8")
        handler.setFormatter(logging.Formatter(
            "Iteration %(iteration)d %(name)s %(message)s"
        ))
        handler.addFilter(RobotIterationFilter(item))
        logger.addHandler(handler)
        item.logger = logger
        logger.info("START position=%s state=%s", (item.x, item.y), item.state)
        log_robot_changes(item, "start")
    return run_directory


def close_robot_logs(robots):
    for item in robots:
        if item.logger is not None:
            log_robot_changes(item, "end")
            item.logger.info("END")
            for handler in item.logger.handlers[:]:
                handler.close()
                item.logger.removeHandler(handler)
            item.logger = None


def open_log_viewers(run_directory):
    terminal = shutil.which("wt.exe")
    powershell = shutil.which("powershell.exe")
    if not powershell:
        print("Live viewers require Windows PowerShell. Logs are still being saved.")
        return
    commands = []
    for path in sorted(run_directory.glob("robot_*.log")):
        # Encode the command so spaces, quotes and shell metacharacters in paths
        # survive both Windows Terminal and PowerShell argument parsing.
        literal_path = str(path.resolve()).replace("'", "''")
        command = f"Get-Content -LiteralPath '{literal_path}' -Tail 100 -Wait -Encoding UTF8"
        encoded = base64.b64encode(command.encode("utf-16-le")).decode("ascii")
        commands.append((path.stem, [powershell, "-NoLogo", "-NoProfile", "-NoExit", "-EncodedCommand", encoded]))
    if not commands:
        return
    try:
        if terminal:
            args = [terminal, "-w", "new"]
            for index, (title, command) in enumerate(commands):
                if index:
                    args += [";", "split-pane"]
                else:
                    args += ["new-tab"]
                args += ["--title", title, "--suppressApplicationTitle", *command]
            subprocess.Popen(args)
        else:
            print("Windows Terminal not found; opening a PowerShell window per robot.")
            for _, command in commands:
                subprocess.Popen(command, creationflags=subprocess.CREATE_NEW_CONSOLE)
    except OSError as error:
        print(f"Could not open log viewers: {error}. Logs are still being saved.")
