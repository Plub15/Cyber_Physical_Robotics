"""Rules checked before a robot performs an action."""

import logging


logger = logging.getLogger(__name__)


def can_move_forward(robot, next_x, next_y):
    """Return whether a robot may move to the requested map position."""
    if robot.world is None:
        return True

    if 0 <= next_x < robot.world.width and 0 <= next_y < robot.world.height:
        return True

    # logger.warning(
    #     "Robot %s ran into the wall",
    #     robot.id,
    # )
    return False

def can_pick_up(robot, attempting_robots=None):
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

    # An explicit set of planned attempts allows validation before any action.
    # Direct callers may still use readyToPick to mark simultaneous attempts.
    if attempting_robots is None:
        attempting_robots = [
            candidate
            for candidate in robot.world.robots.values()
            if candidate is robot or candidate.readyToPick
        ]
    attempting_robots = [
        candidate
        for candidate in attempting_robots
        if robot.world.get_block(candidate.x, candidate.y) is current_block
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


def check_illegal_behavior(actions):
    """Validate the whole action plan without changing robots or the map.

    The mapping is keyed by robot; rejected actions become idle (None).
    Pickup attempts are checked together against the pre-action positions.
    """
    approved_actions = actions.copy()
    pickup_robots = [item for item, action in actions.items() if action == "pick_up"]
    pickup_results = {}

    for item, action in actions.items():
        allowed = True
        if action == "forward":
            delta_x, delta_y = item.DIRECTIONS[item.orientation]
            allowed = can_move_forward(item, item.x + delta_x, item.y + delta_y)
        elif action == "pick_up":
            position = (item.x, item.y)
            if position not in pickup_results:
                pickup_results[position] = can_pick_up(item, pickup_robots)
            allowed = pickup_results[position]
            if item.logger is not None:
                item.logger.info("PICKUP_CHECK position=%s allowed=%s", position, allowed)
        elif action not in (None, "left", "right"):
            raise ValueError(f"Unknown robot action: {action!r}")

        if not allowed:
            approved_actions[item] = None
    return approved_actions
