import math


class block:
    """One cell in a map, containing the entities at that location."""

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.robots = []
        self.targets = []


class map:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.next_robot_id = 0
        self.blocks = [
            [block(x, y) for x in range(width)]
            for y in range(height)
        ]

    def get_block(self, x, y):
        """Return the block containing a position, or None if it is off-map."""
        x = math.floor(x)
        y = math.floor(y)
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.blocks[y][x]
        return None

    def _block_at(self, x, y):
        selected_block = self.get_block(x, y)
        if selected_block is None:
            raise ValueError(f"Position ({x}, {y}) is outside the map.")
        return selected_block

    def add_robot(self, robot):
        selected_block = self._block_at(robot.x, robot.y)
        robot.id = self.next_robot_id
        self.next_robot_id += 1
        selected_block.robots.append(robot)

    def add_target(self, target):
        selected_block = self._block_at(target.x, target.y)
        selected_block.targets.append(target)

    
