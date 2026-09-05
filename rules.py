"""Rules checked before a robot performs an action."""

import logging


logger = logging.getLogger(__name__)


def can_move_forward(robot, next_x, next_y):
    """Return whether a robot may move to the requested map position."""
    if robot.world is None:
        return True

    if robot.world.get_block(next_x, next_y) is not None:
        return True

    logger.warning(
        "Robot %s ran into the wall",
        robot.id,
    )
    return False

def can_pick_up(robot):
    """Return whether a robot may pick up a target"""
    if robot.world is None:
        return False

    current_block = robot.world.get_block(robot.x, robot.y)
    if len(current_block.targets) == 0:
        logger.warning(
            "Robot %s tried to pick up a target where there is none",
            robot.id,
        )
        return False

    return True
