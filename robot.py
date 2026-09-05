import math

from rules import can_move_forward


class robot:
    def __init__(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = math.radians(float(orientation))
        self.id = None
        self.world = None
        self.readyToPick = False
        self.foundTarget = False
        self.inbox = []

    def forward(self):
        """Move the robot forward by a given distance in the direction of its orientation."""
        delta_x = math.cos(self.orientation)
        delta_y = math.sin(self.orientation)

        next_x = self.x + delta_x
        next_y = self.y + delta_y

        if not can_move_forward(self, next_x, next_y):
            return

        self.x = next_x
        self.y = next_y

    def turnLeft(self):
        """Turn the robot left by a given angle."""
        self.orientation += math.radians(90)

    def turnRight(self):
        """Turn the robot right by a given angle."""
        self.orientation -= math.radians(90)

    def pickUp(self):
        pass

    def send(self, id, message):
        if self.world is None:
            raise RuntimeError("Robot is not attached to a world.")

        recipient = self.world.get_robot(id)
        if recipient is None:
            raise ValueError(f"Robot {id} does not exist in this world.")

        payload = {"from": self.id, "message": message}
        recipient.inbox.append(payload)
        recipient._receive()

    def _receive(self):
        if not self.inbox:
            return None

        payload = self.inbox.pop(0)
        print(f"Robot {self.id} received from Robot {payload['from']}: {payload['message']}")
        return payload

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
