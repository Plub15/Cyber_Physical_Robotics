from rules import can_move_forward
import numpy as np



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
        self.foundRobotAhead = False
        self.NumfoundRobotCurrent = 0
        self.RobotIDsInCurrentBlock = []
        self.inbox = []
        self.logger = None
        self.iteration = 0  # Initialization; simulation iterations start at 1.
        self.message_value = 0
        self.goal_x = None
        self.goal_y = None
        self.state = "searching"  # Initial state of the robot
        self.distance_to_goal_value = float("inf")  # Initialize distance to goal as infinity
        self.patience_counter = 10
        self.helper_robot_id = None  # ID of the robot that is helping this robot
        self.helper_robot_distance = float("inf")  # Distance to the helper robot
        self.task_id = None  # ID of the task the robot is currently working on
        self.promise = 0
        self.accepter = 0
        self.response = None
        self.ResponseID = None
        self.command = set()
        self.prevCommand = None
        self.highestMessageValueHeard = 0  # Track the highest message value heard by the robot
        self.commandHeard = None  # Track the command heard by the robot
        self.proposal_ballot = None
        self.promised_ballot = (0, -1)
        self.response_ballot = None
        self.accepted_ballot = None
        self.accepted_value = None
        self.highest_accepted_ballot = None
        self.promise_voters = set()
        self.acceptance_voters = set()

    def forward(self, *, validated=False):
        """Move forward, optionally using approval from the iteration's rule phase."""
        delta_x, delta_y = self.DIRECTIONS[self.orientation]

        next_x = self.x + delta_x
        next_y = self.y + delta_y

        if not validated and not can_move_forward(self, next_x, next_y):
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

    def send(self, message, goal_x=None, goal_y=None, task_id=None, delay=True, prevCommand=None, ResponseID=None, Command=None, max_delay=5, ballot=None, accepted_ballot=None, accepted_value=None):

        #initialize delay
        r = 0

        payload = {"from": self.id, "message": message, "message_value": self.message_value, "position": (self.x, self.y), "goal": None, "distance_to_goal": float("inf"), "helper_robot_id": None, "task_id": task_id, "Command": Command, "prevCommand": prevCommand, "ResponseID": ResponseID, "delay": r}
        payload["ballot"] = ballot
        if ballot is not None:
            payload["message_value"] = ballot[0]
        payload["accepted_ballot"] = accepted_ballot
        if accepted_ballot is not None:
            payload["accepted_value"] = accepted_value
        if goal_x is not None and goal_y is not None:
            payload["goal"] = (goal_x, goal_y)
            payload["distance_to_goal"] = self.distance_to_goal_value
        if self.helper_robot_id is not None:
            payload["helper_robot_id"] = self.helper_robot_id
        for recipient in self.world.robots.values():
            if delay:
                payload["delay"] = np.random.randint(0, max_delay)
            if recipient.id != self.id:
                delivery = payload.copy()
                recipient.inbox.append(delivery)
            else:
                # Send to self
                payload["delay"] = 0  # No delay for self-delivery
                delivery = payload.copy()
                recipient.inbox.append(delivery)
        if self.logger is not None:
            self.logger.debug("SEND to=all-others value=%s message=%r position=%s goal=%s state=%s distance_to_goal=%s helper_robot_id=%s",
                              payload["message_value"], message, payload["position"], payload.get("goal"), self.state,
                              payload.get("distance_to_goal"), payload.get("helper_robot_id"))

    def receive(self,payload):
        if not self.inbox:
            return None

        if payload["delay"] > 0:
            payload["delay"] -= 1
            return None
      
        self.inbox.remove(payload)
        if self.logger is not None:
            self.logger.debug("RECEIVE from=%s value=%s message=%r position=%s goal=%s state=%s distance_to_goal=%s helper_robot_id=%s",
                              payload["from"], payload["message_value"], payload["message"],
                              payload["position"], payload.get("goal"), self.state,
                              payload.get("distance_to_goal"), payload.get("helper_robot_id"))
        return payload

    def sense(self, world):
        # Sense the position in front of the robot based on its orientation
        delta_x, delta_y = self.DIRECTIONS[self.orientation]
        front_x = self.x + delta_x
        front_y = self.y + delta_y

        self.RobotIDsInCurrentBlock = []

        front_block = world.get_block(front_x, front_y)
        self.foundTargetAhead = False
        if front_block is not None:
            if len(front_block.targets) > 0:
                # print(f"Robot {self.id}: Target detected in front of the robot.")
                self.foundTargetAhead = True
            if len(front_block.robots) > 0:
                self.foundRobotAhead = True
                for other_robot in front_block.robots:
                    # print(f"Robot {self.id}: Robot {other_robot.id} detected in front of the robot.")
                    continue
            else:
                self.foundRobotAhead = False

        # Sense the position of the robot
        current_block = world.get_block(self.x, self.y)
        if current_block is not None:
            if len(current_block.robots) > 0:
                self.NumfoundRobotCurrent = len(current_block.robots)
                for other_robot in current_block.robots:
                    if other_robot.id != self.id:
                        # print(f"Robot {self.id}: Robot {other_robot.id} detected in same space as the robot.")
                        self.RobotIDsInCurrentBlock.append(other_robot.id)
                        continue

        # Calculate the distance to the goal if it exists
        self.distance_to_goal_value = self.distance_to_goal()
    def distance_to_goal(self):
        if self.goal_x is None or self.goal_y is None:
            return float("inf")  # Return infinity if the goal is not set
        return  abs(self.x - self.goal_x) + abs(self.y - self.goal_y)
