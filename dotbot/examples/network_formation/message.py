from enum import Enum
from vector2d import Vector2D


# List of possible roles
class RobotState(Enum):
    LEADER = 1
    FOLLOWER = 2
    CONNECTOR = 3
    TRAVELER = 4


# Structure to share the closest connector or distance to a team
class TeamSharedMsg:
    def __init__(self) -> None:
        self.dist = None # float
        self.connector_id_upstream = '' # str
        self.connector_id_downstream = '' # str


# Structure to store the connection to the team
class HopMsg:
    def __init__(self) -> None:
        self.count = None # int
        self.id = '' # str
        self.resend_count = 100000 # int


# Structure to store the network change message
class NetworkChangeMsg:
    def __init__(self) -> None:
        self.team_id = None # int
        self.prev_id = None # str
        self.new_id = None # str

# Structure to store request/approval messages for extending the network
class ConnectionMsg:
    def __init__(self) -> None:
        self.type = 'N'  # 'R'-Request, 'A'-Accept, 'N'-None
        self.origin = None # str
        self.to = None # str
        self.to_team = None # int
        self.other_team = None # int


# Message sent by a leader to other leaders
class RelayMsg:
    def __init__(self) -> None:
        self.type = 'H' # 'H'-HeartBeat, 'R'-RequestRobot , 'A'-Acknowledge
        self.origin = None
        self.origin_team = None
        self.time = None # float
        self.first_follower = None # str
        self.first_follower_dist = 0 # float
        self.follower_num = None # int
        self.task_min_num = 0 # int
        self.robot_num = 0 # int
        self.request_to_team = None # int
        self.accept_to_team = None # int


# Structure to store incoming data received from other robots
class Message:
    def __init__(self) -> None:
        # Core
        self.direction = Vector2D(0,0) # Vector2D
        self.state = None # RobotState
        self.id = None # str
        self.team_id = None # int

        # self.leader_signal = None # int

        self.robot_to_switch = None # str
        self.team_to_join = None # int

        self.team_hop_count = 100000 # int

        self.tmsg = {} # dict (key: teamID, val: TeamSharedMsg)

        self.hops = {} # dict (key: teamID, val: HopMsg)

        self.nmsg = [] # list (NetworkChangeMsg)

        self.cmsg = [] # list (ConnectionMsg)

        self.nearby_teams = {} # dict (key: teamID, val: dist)

        self.rmsg = [] # list (RelayMsg)

        self.connections = [] # list (ID)


    # Checks whether the Message is empty or not by checking the direction it was received from
    def empty(self) -> bool:
        return self.id == None
