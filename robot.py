from rules import can_move_forward


class robot:
    # East, north, west, south.  The index is the robot's discrete orientation.
    DIRECTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))

    def __init__(self, x, y, orientation):
        self.x = x
        self.y = y
        orientation = float(orientation)
        if orientation % 90 != 0:
            raise ValueError("Robot orientation must be a multiple of 90 degrees.")
        self.orientation = int(orientation // 90) % len(self.DIRECTIONS)
        self.id = None
        self.world = None
        self.readyToPick = False
        self.foundTargetAhead = False
        self.foundTargetCurrent = False
        self.inbox = []
        self.logger = None
        self.message_value = 0
        self.goal_x = None
        self.goal_y = None
        self.state = "searching"  # Initial state of the robot

    def forward(self):
        """Move the robot forward by a given distance in the direction of its orientation."""
        delta_x, delta_y = self.DIRECTIONS[self.orientation]

        next_x = self.x + delta_x
        next_y = self.y + delta_y

        if not can_move_forward(self, next_x, next_y):
            return
        else:
            self.world.move_robot(self, next_x, next_y)

    def turnLeft(self):
        """Turn the robot left by a given angle."""
        self.orientation = (self.orientation + 1) % len(self.DIRECTIONS)

    def turnRight(self):
        """Turn the robot right by a given angle."""
        self.orientation = (self.orientation - 1) % len(self.DIRECTIONS)

    def pickUp(self):
        self.readyToPick = True
        pass

    def send(self, message, goal_x=None, goal_y=None):

        payload = {"from": self.id, "message": message, "message_value": self.message_value, "position": (self.x, self.y)}
        if goal_x is not None and goal_y is not None:
            payload["goal"] = (goal_x, goal_y)
        for recipient in self.world.robots.values():
            if recipient.id != self.id:
                recipient.inbox.append(payload)
        if self.logger is not None:
            self.logger.debug("SEND to=all-others value=%s message=%r position=%s goal=%s state=%s",
                              payload["message_value"], message, payload["position"], payload.get("goal"), self.state)
        self.message_value += 1

    def receive(self):
        if not self.inbox:
            return None

        payload = self.inbox.pop(-1)
        if self.logger is not None:
            self.logger.debug("RECEIVE from=%s value=%s message=%r position=%s goal=%s state=%s",
                              payload["from"], payload["message_value"], payload["message"],
                              payload["position"], payload.get("goal"), self.state)
        self.message_value = max(self.message_value, payload["message_value"] + 1)
        return payload

    def sense(self, world):
        # Sense the position in front of the robot based on its orientation
        delta_x, delta_y = self.DIRECTIONS[self.orientation]
        front_x = self.x + delta_x
        front_y = self.y + delta_y

        front_block = world.get_block(front_x, front_y)
        self.foundTargetAhead = False
        if front_block is not None:
            if len(front_block.targets) > 0:
                # print(f"Robot {self.id}: Target detected in front of the robot.")
                self.foundTargetAhead = True
            if len(front_block.robots) > 0:
                for other_robot in front_block.robots:
                    # print(f"Robot {self.id}: Robot {other_robot.id} detected in front of the robot.")
                    continue

        # Sense the position of the robot
        current_block = world.get_block(self.x, self.y)
        if current_block is not None:
            if len(current_block.targets) > 0:
                # print(f"Robot {self.id}: Target is on the robot.")
                self.foundTargetCurrent = True
            else:
                self.foundTargetCurrent = False
            if len(current_block.robots) > 0:
                for other_robot in current_block.robots:
                    if other_robot.id != self.id:
                        # print(f"Robot {self.id}: Robot {other_robot.id} detected in same space as the robot.")
                        continue
