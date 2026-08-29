import math

class robot:
    def __init__(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = math.radians(float(orientation))
        self.id = None
        self.readyToPick = False
        self.foundTarget = False

    def forward(self):
        """Move the robot forward by a given distance in the direction of its orientation."""
        self.x += 1 * math.cos(self.orientation)
        self.y += 1 * math.sin(self.orientation)

    def turnLeft(self):
        """Turn the robot left by a given angle."""
        self.orientation += math.radians(90)

    def turnRight(self):
        """Turn the robot right by a given angle."""
        self.orientation -= math.radians(90)

    def pickUp(self):
        pass

    def _sense(self, world):
        """Return True if there is a target in front of the robot, False otherwise."""
        # Calculate the position in front of the robot based on its orientation
        front_x = self.x + math.cos(self.orientation)
        front_y = self.y + math.sin(self.orientation)

        front_block = world.get_block(front_x, front_y)
        if front_block is not None:
            if len(front_block.targets) > 0:
                print(f"Robot {self.id}: Target detected in front of the robot.")
            if len(front_block.robots) > 0:
                for other_robot in front_block.robots:
                    print(f"Robot {self.id}: Robot {other_robot.id} detected in front of the robot.")

        current_block = world.get_block(self.x, self.y)
        if current_block is not None:
            if len(current_block.targets) > 0:
                print(f"Robot {self.id}: Target is on the robot.")
            if len(current_block.robots) > 0:
                for other_robot in current_block.robots:
                    if other_robot.id != self.id:
                        print(f"Robot {self.id}: Robot {other_robot.id} detected in same space as the robot.")
        