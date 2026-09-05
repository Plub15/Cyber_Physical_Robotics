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
    """Return whether exactly two robots are attempting the same pickup."""
    if robot.world is None:
        return False

    current_block = robot.world.get_block(robot.x, robot.y)
    if current_block is None or len(current_block.targets) == 0:
        logger.warning(
            "Robot %s tried to pick up a target where there is none",
            robot.id,
        )
        return False

    # Calling this rule is itself an attempt. Other simultaneous attempts are
    # represented by readyToPick and must be in the same block as the target.
    attempting_robots = [
        candidate
        for candidate in robot.world.robots.values()
        if (candidate is robot or candidate.readyToPick)
        and robot.world.get_block(candidate.x, candidate.y) is current_block
    ]
    attempt_count = len(attempting_robots)

    if attempt_count == 1:
        logger.warning(
            "Robot %s tried to pick up a target alone; exactly two robots are required",
            robot.id,
        )
        return False

    if attempt_count > 2:
        logger.warning(
            "%s robots tried to pick up the same target; exactly two robots are required",
            attempt_count,
        )
        return False

    return attempt_count == 2
