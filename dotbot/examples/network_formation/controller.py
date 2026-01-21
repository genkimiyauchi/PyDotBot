from enum import Enum
import time
import math
from copy import deepcopy
from collections import defaultdict
import queue
from vector2d import Vector2D
import xml.etree.ElementTree as ET

from message import RobotState, Message, HopMsg, NetworkChangeMsg, TeamSharedMsg, ConnectionMsg, RelayMsg
from dotbot.examples.sct import SCTPub

from colorama import Fore

# E-puck proximity sensor angles
PROX_ANGLE = [
                math.pi / 10.5884,
                math.pi / 3.5999,
                math.pi / 2,        # side sensor
                math.pi / 1.2,      # back sensor
                math.pi / 0.8571,   # back sensor
                math.pi / 0.6667,   # side sensor
                math.pi / 0.5806,
                math.pi / 0.5247
            ]

# TEMP
# Software fix for sensors returning values that are too high
# key = robot_id, val = list(sensor number, amount to modify from raw reading)
SENSOR_CALIBRATION = {
                        13: [(5, -100)],
                        16: [(2, -100)]
                    }

# Robot teleopration states to use in the controller
class TeleopState(Enum):
    FORWARDS = 1
    BACKWARDS = 2
    LEFT = 3
    RIGHT = 4
    STOP = 5
    FORWARDS_LEFT = 6
    FORWARDS_RIGHT = 7
    BACKWARDS_LEFT = 8
    BACKWARDS_RIGHT = 9


# List of move types available to the worker
class MoveType(Enum):
    FLOCK = 1
    ADJUST = 2
    TRAVEL = 3


# Structure to store the set of conditions to determine whether it
# can switch its role (either to a follower or a connector).
class RoleSwitchConditions:
    
    def __init__(self) -> None:
        # (F -> C) Whether distance exceeds the threshold
        self.c1 = False
        # (F -> C) Whether it is the closest among its teammates
        self.c2 = False
        # (F -> C) Whether the distance between the leader and connection candidate exceeds the separation threshold
        self.c3 = False
        # (C -> F) Whether it is a tail connector
        self.f1 = False
        # (c -> F) Whether the team is closer to its successor in the chain
        self.f2 = False


    def connector_conditions(self):
        """Whether the conditions to become a connector are satisfied for this team"""
        return self.c1 and self.c2 and self.c3


# Wheel params
base_angular_wheel_speed = 100

# Get vector from range and bearing
def get_vector(robot):
    x = robot.position.x
    y = robot.position.y
    return Vector2D(x, y)


# # Get distance to a robot
# def get_distance_to_robot(robot):
#     vec = get_vector(robot)
#     return abs(vec)


# Leader IDs
leader_id = {}


# Main Robot class to keep track of robot states
class Robot:

    # 3.6V should give an indication that the battery is getting low, but this value can be experimented with.
    # Battery percentage might be a better
    BAT_LOW_VOLTAGE = 3.6

    # Firmware on both robots accepts wheel velocities between -100 and 100.
    # This limits the controller to fit within that.
    MAX_SPEED = 100

    def __init__(self, robot_id, config=None, team_id=1):
        self.id = robot_id
        self.connection = None

        self.orientation = 0
        self.neighbours = {}
        self.tasks = {}

        self.teleop = False
        self.teleop_state = TeleopState.STOP
        self.ir_readings = []
        self.battery_charging = False
        self.battery_voltage = 0
        self.battery_percentage = 0

        self.turn_time = time.time()

        self.ir_threshold = 200

        self.led_colour = (0,0,0) # off

        ### Framework related (common) ###

        # Core
        self.current_state = RobotState.FOLLOWER # RobotState
        self.team_id = team_id # int
        self.msg = Message()

        self.init_step_timer = 0

        # # SCT
        self.sct = None

        # Message received from nearby robots
        self.messages = {}
        self.team_msgs = []
        self.connector_msgs = []
        self.other_leader_msgs = []
        self.other_team_msgs = []
        self.traveler_msgs = []

        # Team Shared Message
        self.team_shared_msg_dict = defaultdict(TeamSharedMsg)

        # Number of hops to each team
        self.hops_dict = {} 

        # Connection related info to send in the current timestep
        self.cmsg_to_send = []
        # ConnectionMsg attached with a timer. Messages ets added into cmsg_to_send while timer is running 
        self.cmsg_to_resend = [] # list of pairs (timer, ConnectionMsg)

        self.rmsg_to_send = []
        self.rmsg_to_resend = [] # list of pairs (timer, RelayMsg)

        self.log_messages = [] #  list of log strings


    def set_wheel_speeds(self, sum_force):
        heading_angle = math.atan2(sum_force.y, sum_force.x)
        heading_length = abs(sum_force)

        print('angle: {}'.format(heading_angle))
        print('length: {}'.format(heading_length))

        speed_factor = (1.57 - abs(heading_angle)) / 1.57
        speed1 = base_angular_wheel_speed + base_angular_wheel_speed * (1.0 - speed_factor)
        speed2 = base_angular_wheel_speed - base_angular_wheel_speed * (1.0 - speed_factor)

        # print("speed_factor: {}".format(speed_factor))
        # print("speed1: {}".format(speed1))
        # print("speed2: {}".format(speed2))

        if(heading_angle > 0):
            # Turn left
            left  = speed1
            right = speed2
        else:
            # Turn right
            left  = speed2
            right = speed1

        # print('left: {}'.format(left))
        # print('right: {}'.format(right))

        return left, right


    # def get_obstacle_repulsion_vector(self):

    #     res_vec = Vector2D(0,0)

    #     # Apply fixed value to faulty sensors
    #     if self.id in SENSOR_CALIBRATION:
    #         for sensor_id, value in SENSOR_CALIBRATION[self.id]:
    #             self.ir_readings[sensor_id] += value

    #     # Print proximity readings
    #     print('[', end='')
    #     for i in range(len(self.ir_readings)):
    #         if self.ir_readings[i] > self.ir_threshold:
    #             print(Fore.LIGHTRED_EX + f'{self.ir_readings[i]}', end='')
    #         else:
    #             print(Fore.LIGHTGREEN_EX + f'{self.ir_readings[i]}', end='')
    #         if i < len(self.ir_readings) - 1:
    #             print(', ', end='')
    #     print(']')

    #     # print(Fore.LIGHTGREEN_EX + f'{self.ir_readings}')

    #     count = 0

    #     for i in range(len(self.ir_readings)):

    #         if self.ir_readings[i] > self.ir_threshold:
    #             length = 1
    #             x = math.cos(PROX_ANGLE[i]) * length
    #             y = math.sin(PROX_ANGLE[i]) * length
    #             vec = Vector2D(x,y)

    #             res_vec -= vec # subtract because we want the vector to repulse from the obstacle

    #             count += 1

    #     if count > 0:
    #         res_vec /= count

    #     # print(res_vec)

    #     return res_vec


    def get_obstacle_repulsion_vector(self):

        res_vec = Vector2D(0,0)

        count = 0

        # Loop all neighbors
        for key, value in self.neighbours.items():
            
            distance = value.range

            # If it is too close, add a unit vector in the opposite direction
            if distance < 0.11:
                length = 1
                x = math.cos(math.radians(value.bearing)) * length
                y = math.sin(math.radians(value.bearing)) * length
                rep_vec = Vector2D(x, y)
                res_vec -= rep_vec # subtract because we want the vector to repulse from the obstacle
 
                count += 1
                print(count)

        # Average the total vector
        if count > 0:
            res_vec /= count

        # print(res_vec)

        return res_vec


class Leader(Robot):

    # Class parameters

    # Flocking params
    TARGET_DISTANCE = 0.08
    GAIN = 100
    EXPONENT = 2

    TARGET_ANGLE = 0
    WAYPOINT_TRACKING_KP = 10
    WAYPOINT_TRACKING_KI = 0
    WAYPOINT_TRACKING_KD = 0
    THRES_RANGE = 0.001

    MIN_DISTANCE_FROM_ROBOT = 0.45
    SEPARATION_THRES = 0.3

    SEND_DURATION = 4
    SEND_ROBOT_DELAY = 30

    SCT_PATH = 'dotbot/examples/network_formation/models/leader_nop.yaml'

    @classmethod
    def load_params(cls, config):
        
        for param in config[0]:
            # if param =='team_flocking':
            #     cls.TARGET_DISTANCE = param.get('target_distance')
            #     cls.GAIN = param.get('gain')
            #     cls.EXPONENT = param.get('exponent')

            if param.tag == 'waypoint_tracking':
                cls.TARGET_ANGLE = float(param.get('target_angle'))
                cls.WAYPOINT_TRACKING_KP = float(param.get('kp'))
                cls.WAYPOINT_TRACKING_KI = float(param.get('ki'))
                cls.WAYPOINT_TRACKING_KD = float(param.get('kd'))
                cls.THRES_RANGE = float(param.get('thres_range'))

            elif param.tag =='team_flocking':
                cls.TARGET_DISTANCE = float(param.get('target_distance'))
                cls.GAIN = float(param.get('gain'))
                cls.EXPONENT = float(param.get('exponent'))

            elif param.tag == 'team_distance':
                cls.MIN_DISTANCE_FROM_ROBOT = float(param.get('min_leader_robot_distance'))
                cls.SEPARATION_THRES = float(param.get('separation_threshold'))

            elif param.tag == 'timeout':
                cls.SEND_DURATION = float(param.get('send_message'))
                cls.SEND_ROBOT_DELAY = float(param.get('send_robot_delay'))

            elif param.tag == 'SCT':
                cls.SCT_PATH = param.get('path')


    def __init__(self, robot_id, config=None, team_id=1):
        super().__init__(robot_id, config, team_id)

        self.id = self.id

        # self.left = 0
        # self.right = 0
        self.current_goal = Vector2D(0,0)

        self.led_colour = (255,0,0) # red

        ### Framework related (leader) ###

        self.current_state = RobotState.LEADER
        leader_id[self.team_id] = self.id

        # Add SCT callback functions
        self.sct = SCTPub(Leader.SCT_PATH)
        self.add_callbacks()

        self.signal = False # Whether the leader is sending a task signal

        # Flag to indicate triggering of uncontrollable
        self.input_start = True # Input received from the user. TEMP: Make leader always send start signal for now
        self.input_stop = False
        self.received_request = False

        # Respond to follower message request
        self.accept_ids = {}

        self.current_task_min_robot_num = 0
        self.current_follower_count = 0
        self.num_other_follower = -1
        self.num_other_task_require = 0
        self.last_sent = -1
        self.beat_sent = 0
        
        self.last_beat_time = {} # {team_id: last_time}
        self.beat_received = {} # {team_id: message_count}

        self.num_robots_to_send = 0
        self.num_robots_remaining_to_send = 0
        self.num_robots_to_request = 0
        self.num_robots_remaining_to_request = 0 # Used during the behavior analysis experiment
        self.all_requests_satisfied = False # Used during the behavior analysis experiment
        self.all_requests_satisfied_time = 0 # Used during the behavior analysis experiment
        self.num_robots_requested = {} # Number of robots requested by other teams
        self.robot_to_switch = ''
        self.team_to_join = None
        self.switch_candidate = '' # Robot that the leader could choose to switch to the other team
        self.switch_candidate_dist = 100000 # Distance of the candidate to the non-team robot it received the message from
        self.switch_candidate_timer = 0 # Time it recorded the switch_candidate
        self.request_sent = False
        self.acknowledge_sent = False
        self.other_teams = set()
        self.teams_to_request = set() # Used to request to teams in order
        self.decremented = False
        self.robot_last_sent_time = 0 # Timer to count the number of messages to send before sending the next robot
        self.robot_last_sent = None # ID of robot last sent

        # FOR WAYPOINT TRACKING
        self.position = Vector2D(0,0)
        self.waypoints = queue.Queue()
        self.in_task = False

        # Info about the current task when it is inside the task range
        self.current_task_init_demand = 0
        self.current_task_demand = 0
        self.current_task_min_robot_num = 0
        self.current_task_id = ''

        # self.ex1_11_StartCondition = False # For swarmhack, make pi-puck 11 be the leader of team 1

        # # TEMP: hard coded team to join (Assuming two teams)
        # if self.team_id == 1:
        #     self.team_to_join = 2
        # elif self.team_id == 2:
        #     self.team_to_join = 1


    # Register callback functions to the generator player
    def add_callbacks(self):

        # Automatic addition of callbacks
        # 1. Get list of events and list specifying whether an event is controllable or not.
        # 2. For each event, check controllable or not and add callback.

        events, controllability_list = self.sct.get_events()

        for event, index in events.items():
            is_controllable = controllability_list[index]
            stripped_name = event.split('EV_', 1)[1]    # Strip preceding string 'EV_'

            if is_controllable: # Add controllable event
                func_name = '_callback_{0}'.format(stripped_name)
                func = getattr(self, func_name)
                self.sct.add_callback(event, func, None, None)
            else:   # Add uncontrollable event
                func_name = '_check_{0}'.format(stripped_name)
                func = getattr(self, func_name)
                self.sct.add_callback(event, None, func, None)


    # Flocking repulsion
    def generalized_lennard_jones_repulsion(self, distance):
        f_norm_dist_exp = pow(Leader.TARGET_DISTANCE / distance, Leader.EXPONENT)
        return -Leader.GAIN / distance * (f_norm_dist_exp * f_norm_dist_exp)


    def control_step(self):
        
        self.init_step_timer += 1

        # DEBUG
        print(Fore.LIGHTCYAN_EX + f'--- Robot {self.id} ---')
        print(Fore.LIGHTCYAN_EX + f'state: {self.current_state}, id: {self.id}, team id: {self.team_id}, time: {self.init_step_timer}')

        self.reset_variables()

        self.get_messages()
        self.update()

        # Run SCT controller
        if self.init_step_timer > 4:
            self.sct.run_step()
        # print(f'Supervisors: {self.sct.sup_current_state}')

        # TODO
        # FOR REAL HUMAN CONTROL

        # FOR SIMULATED HUMAN CONTROL
        if self.init_step_timer > 10: # wait for a while before moving towards the tasks
            self.current_goal = self.waypoint_tracking()
            print(f'current_goal: {self.current_goal}, {self.init_step_timer}')

        # Create message
        msg = Message()
        msg.state = self.current_state
        msg.id = self.id
        msg.team_id = self.team_id
        # msg.leader_signal = 1 # TEMP, always signal to work on tasks

        if self.robot_to_switch:
            msg.robot_to_switch = self.robot_to_switch
            msg.team_to_join = self.team_to_join

        # Team hop count
        msg.team_hop_count = 0

        # Set team shared message
        msg.tmsg = deepcopy(self.team_shared_msg_dict)

        # Set network hop counts
        msg.hops = deepcopy(self.hops_dict)
        for id in msg.hops:
            msg.hops[id].resend_count += 1

        # Connection Message
        for resend_msg in self.cmsg_to_resend:
            self.cmsg_to_send.append(resend_msg[1])
            resend_msg[0] -= 1 # Decrement timer

        for cmsg in self.cmsg_to_resend:
            if cmsg[0] <= 0:
                if cmsg[1].other_team in self.accept_ids:
                    self.accept_ids.pop(cmsg[1].other_team)

        self.cmsg_to_resend = [x for x in self.cmsg_to_resend if x[0] > 0] # Remove messages that have timed out
        # print(f'cmg_to_resend: {self.cmsg_to_resend}')

        msg.cmsg = deepcopy(self.cmsg_to_send)

        # Relay Message
        self.rmsg_to_resend = [x for x in self.rmsg_to_resend if x[0] > 0] # Remove messages that have timed out
        for resend_msg in self.rmsg_to_resend:
            self.rmsg_to_send.append(resend_msg[1])
            resend_msg[0] -= 1 # Decrement timer

        msg.rmsg = deepcopy(self.rmsg_to_send)

        # Set message to send in this timestep
        self.msg = msg

        # Delete old messages
        self.messages.clear()

        print(f'switch_candidate: {self.switch_candidate}')

        print()


    def reset_variables(self):
        self.team_msgs.clear()
        self.connector_msgs.clear()
        self.other_leader_msgs.clear()
        self.other_team_msgs.clear()
        self.traveler_msgs.clear()

        self.cmsg_to_send.clear()
        self.rmsg_to_send.clear()

        self.team_shared_msg_dict.clear()

        self.in_task = False

        self.received_request = False

        self.robot_to_switch = ''


    def set_robots_to_request(self, num_robots):
        print(Fore.YELLOW + f"Received {num_robots} to request from user")
        self.num_robots_to_request = int(num_robots)


    def set_robots_to_send(self, num_robots):
        print(Fore.YELLOW + f"Received {num_robots} to send from user")

        if self.current_follower_count <= 1:
            print(Fore.YELLOW + '[LOG] Cannot send if robots <= 1')
            self.log_messages.append('[LOG] Cannot send if robots <= 1')
        elif self.current_follower_count <= num_robots: # If robots to send exceed current team size, send all followers
            self.num_robots_to_send = self.current_follower_count - 1
        else:
            self.num_robots_to_send = int(num_robots)

        self.num_robots_remaining_to_send = self.num_robots_to_send


    def get_messages(self):
        print(f'messages: {self.messages}')

        print(f'self.neighbours: {self.neighbours}')

        for id, msg in self.messages.items():

            # Set vector pointing at neighbor (with respect to its own local frame)
            if str(id) in self.neighbours:
                msg.direction = get_vector(self.neighbours[str(id)])

            if msg.state == RobotState.LEADER:
                self.other_leader_msgs.append(msg)
            elif msg.state == RobotState.FOLLOWER:
                if msg.team_id == self.team_id:
                    self.team_msgs.append(msg)
                else:
                    self.other_team_msgs.append(msg)
            elif msg.state == RobotState.CONNECTOR:
                self.connector_msgs.append(msg)
            elif msg.state == RobotState.TRAVELER:
                self.traveler_msgs.append(msg)

        # DEBUG
        # for id, msg in self.messages.items():
        #     print(f"id: {id}, team id: {msg.team_id}, leader signal: {msg.leader_signal}, team hop count: {msg.team_hop_count}, to_switch: {msg.robot_to_switch}, to_team: {msg.team_to_join}")

        #     for team_id, hop in msg.hops.items():
        #         print(f'\t[hop] team_id: {team_id}, id: {hop.id}, count: {hop.count}, resend_count: {hop.resend_count}')
        #     for team_id, tmsg in msg.tmsg.items():
        #         print(f'\t[tmsg] team_id: {team_id}, dist: {tmsg.dist}, up: {tmsg.connector_id_upstream}, down: {tmsg.connector_id_downstream}')
        #     for cmsg in msg.cmsg:
        #         print(f'\t[cmsg] type: {cmsg.type}, from: {cmsg.origin}, to: {cmsg.to}, to_team: {cmsg.to_team}')
        #     for rmsg in msg.rmsg:
        #         print(f'\t[rmsg] type: {rmsg.type}, from: {rmsg.origin}, time: {rmsg.time}, robot_num: {rmsg.robot_num}')


    def update(self):
        
        # near_robot = self.is_near_robot()

        # Update knowledge of all teams
        for team, hop in self.hops_dict.items():
            self.other_teams.add(team)
        # Refill teams that it can send a request to
        if not self.teams_to_request:
            self.teams_to_request = deepcopy(self.other_teams)

        self.update_hop_counts()
        self.set_connector_to_relay()
        # self.reply_to_request()
        self.check_heart_beat()
    

    def update_hop_counts(self):
        """Update hop count within team and to other teams"""
        
        # Get Connector messages that has connection to this team
        adjacent_connector_msgs = []
        for msg in self.connector_msgs:
            for id, hop in msg.hops.items():
                if id == self.team_id:
                    if hop.id == "" or hop.id in leader_id.values():
                        adjacent_connector_msgs.append(msg)
                        break

        # Get team IDs from team and adjacent connectors
        combined_msgs = adjacent_connector_msgs + self.team_msgs
        team_ids = set()
        for msg in combined_msgs:
            for id, hop in msg.hops.items():
                team_ids.add(id)
        if self.team_id in team_ids:
            team_ids.remove(self.team_id) # Delete its own team id

        # print(f'team ids: {team_ids}')

        for other_team_id in team_ids:

            # Loop connector (that has connections to this team)

            connector_found = False
            
            for msg in adjacent_connector_msgs:

                # If connector is a predecessor, use it to update its own hop count
                if other_team_id in msg.hops:
                    if msg.hops[other_team_id].id == '' or not msg.hops[other_team_id].id in leader_id.values():
                        
                        if not other_team_id in self.hops_dict:
                            self.hops_dict[other_team_id] = HopMsg()

                        self.hops_dict[other_team_id].count = msg.hops[other_team_id].count + 1
                        self.hops_dict[other_team_id].id = msg.id
                        self.hops_dict[other_team_id].resend_count = 0
                        connector_found = True
                        break

            if not connector_found:
                # Loop teammates
                combined_team_msgs = self.team_msgs.copy()

                min_hop = HopMsg()
                min_resend_count = 100000
                for msg in combined_team_msgs:
                    if other_team_id in msg.hops and msg.hops[other_team_id].resend_count < min_resend_count:
                        min_hop = msg.hops[other_team_id]
                        min_resend_count = msg.hops[other_team_id].resend_count

                if min_resend_count < 100000:
                    self.hops_dict[other_team_id] = min_hop
        
        # print(f'self.hops_dict: {self.hops_dict}')
        print(Fore.LIGHTBLUE_EX + f'hops_dict: ', end='')
        for id, hop in self.hops_dict.items():
            print(Fore.LIGHTBLUE_EX + f'({id}, id:{hop.id}, hop:{hop.count}, resend:{hop.resend_count})', end='')
        print()
        

    def set_connector_to_relay(self):

        # Get all visible teams from connectors and followers
        combined_msgs = self.connector_msgs + self.team_msgs

        team_ids = set() # set of team ids
        for msg in combined_msgs:
            for id, hop in msg.hops.items():
                team_ids.add(id)

        if self.team_id in team_ids:
            team_ids.remove(self.team_id) # Delete its own team id

        # Add new entries to team_shared_msg_dict if it doesn't exist
        for id in team_ids:
            if not id in self.team_shared_msg_dict:
                self.team_shared_msg_dict[id] = TeamSharedMsg()

        # Update connector to relay to the team
        for other_team_id, team_msg in self.team_shared_msg_dict.items():
            
            # # Check if it detects the tail connector directly
            # connector_nearby = False

            # min_hop_count = 100000
            # min_hop_count_robot = ''

            # for msg in self.connector_msgs:
            #     # Find connector with the smallest hop count to the team and record it
            #     for id, hop in msg.hops.items():
            #         if id == other_team_id:
            #             if min_hop_count_robot == '' or hop.count < min_hop_count:
            #                 min_hop_count = hop.count
            #                 min_hop_count_robot = msg.id
            #                 connector_nearby = True

            # if connector_nearby:
            #     self.team_shared_msg_dict[other_team_id].connector_id_downstream = min_hop_count_robot # Update ID to send downstream to team
            # else:
            if self.team_msgs:
                previous_seen = False
                new_value = False

                for msg in self.team_msgs:
                    new_info_found = False
                    for id, follower_team_msg in msg.tmsg.items():
                        if id == other_team_id:

                            # if follower_team_msg.connector_id_upstream:
                            if follower_team_msg.connector_id_upstream == self.team_shared_msg_dict[other_team_id].connector_id_downstream:

                                previous_seen = True # Received the same connector as before. Confirms current data is correct

                            else:

                                # Update connector info
                                self.team_shared_msg_dict[other_team_id].connector_id_downstream = follower_team_msg.connector_id_upstream
                                new_value = True
                                new_info_found = True
                                break

                    if new_info_found:
                        break

                # print(f'previous_seen: {previous_seen}')
                # print(f'new_value: {new_value}')
                if not previous_seen and not new_value: # If previous info not received and no new info, reset downstream
                    self.team_shared_msg_dict[other_team_id].connector_id_downstream = ''

            else:
                # Found and received no info about this tea, so reset downstream
                self.team_shared_msg_dict[other_team_id].connector_id_downstream = ''

        # print(f'self.team_shared_msg_dict: {self.team_shared_msg_dict}')


    def check_heart_beat(self):
        
        combined_msgs = self.other_leader_msgs + self.team_msgs + self.other_team_msgs + self.connector_msgs

        for msg in combined_msgs:
            for beat in msg.rmsg:
                if beat.origin != self.id:

                    # Get team ID
                    received_team_id = beat.origin_team

                    # If receiving this team for the first time, initialize
                    if not received_team_id in self.last_beat_time:
                        self.last_beat_time[received_team_id] = 0
                        self.beat_received[received_team_id] = 0

                    if beat.time > self.last_beat_time[received_team_id]:

                        # Update
                        self.last_beat_time[received_team_id] = beat.time
                        self.beat_received[received_team_id] += 1

                        if msg.state == RobotState.LEADER:
                            print(Fore.YELLOW + f'received_message')
                        else:
                            print(Fore.YELLOW + f'received_relay')

                        self.decremented = False

                        # Store message info from other leader
                        self.num_other_follower = beat.follower_num
                        self.num_other_task_require = beat.task_min_num

                        if beat.type == 'R':
                            print(Fore.LIGHTYELLOW_EX + f'TYPE Request received')
                            if beat.request_to_team == self.team_id:
                                self.num_robots_requested[received_team_id] = beat.robot_num
                                self.team_to_join = received_team_id # TEMP: Assuming only one team ever requests (behavior analysis)
                                print(Fore.YELLOW + f'[REQUEST] Received request to send {beat.robot_num} robots')
                                self.log_messages.append(f'[REQUEST] Received request to send {beat.robot_num} robots')
                            
                        elif beat.type == 'A' and beat.accept_to_team == self.team_id:
                            print(f'{self.id} Received Acknowledge from {beat.origin} who is sending {beat.robot_num}')
                            if beat.robot_num > 0:
                                print(f'[SEND] {beat.robot_num} robots are heading this way!')
                                self.log_messages.append(f'[SEND] {beat.robot_num} robots are heading this way!')

                                if self.num_robots_remaining_to_request > 0:
                                    if beat.robot_num >= self.num_robots_remaining_to_request:
                                        self.num_robots_remaining_to_request = 0 # No more robots to request
                                    else:
                                        self.num_robots_remaining_to_request -= beat.robot_num # Update remaining robots to request to other teams
                                    
                                    self.all_requests_satisfied = True
                                    self.all_requests_satisfied_time = self.init_step_timer

                                self.request_sent = False
                            else:
                                print(f'{self.id}[SEND] Request to team {received_team_id} was rejected')
                                self.request_sent = False

                        # self.switch_candidate = '' # Reset candidate follower to switch

                    # Set a follower that received the leader message from a non-team robot as a candidate to switch teams
                    if beat.first_follower and msg.team_id == self.team_id and msg.id != self.robot_last_sent:
                        update = False
                        if not self.switch_candidate:
                            update = True
                        elif beat.first_follower_dist < self.switch_candidate_dist:
                            update = True
                        elif self.switch_candidate_timer + 10 < self.init_step_timer:
                            update = True

                        if update:
                            self.switch_candidate = beat.first_follower
                            self.switch_candidate_dist = beat.first_follower_dist
                            self.switch_candidate_timer = self.init_step_timer


    def waypoint_tracking(self):
        
        # TODO Stop if other robots are too far

        if not self.waypoints.empty():

            # if self.id != 11 or self.ex1_11_StartCondition:

            # Check if it is near the waypoint
            dist = list(self.waypoints.queue)[0].distance_to(self.position)
            self.in_task = dist <= Leader.THRES_RANGE

            if self.current_task_demand > 0:
                print(Fore.RED + f'Current demand: {self.current_task_demand}')
            else:
                print(Fore.GREEN + f'Current demand: {self.current_task_demand}')

            # If current task is completed, move to the next one
            print(f'Distance to task: {dist}')

            if not self.in_task:

                # repulse_msgs = []
                # repulse_msgs += self.other_leader_msgs + self.other_team_msgs
                # # for msg in self.connector_msgs:
                # #     if msg.hops[self.team_id] != 1: # Do not repulse from connector that is directly connected to its team
                # #         repulse_msgs.append(msg)
                # repulse_ids = {str(msg.id) for msg in repulse_msgs}

                waypoint_force = self.vector_to_waypoint()
                # robot_force = self.get_robot_repulsion_vector(repulse_ids)
                # obstacle_force = self.get_obstacle_repulsion_vector()

                # waypoint_weight = 1
                # robot_weight = 0.03
                # obstacle_weight = 1

                # sum_force = waypoint_weight * waypoint_force + robot_weight * robot_force + obstacle_weight * obstacle_force
                # print("obstacle vector: {}".format(obstacle_force))

                waypoint_force = list(self.waypoints.queue)[0]

                print("Sum vector: {}".format(waypoint_force))

                # if abs(waypoint_force) > 0.01:
                return waypoint_force
                # else:
                #     return None

            else:
                if self.current_task_demand == 0:
                    self.waypoints.get() # pop first waypoint
                return None
        else:
            self.in_task = True
            return None


    def vector_to_waypoint(self):
        
        # Get current position
        vector_to_waypoint = list(self.waypoints.queue)[0] - self.position
        print(f'waypoint: {list(self.waypoints.queue)[0]}')
        print(f'position: {self.position}')
        print(f'vector_to_waypoint: {vector_to_waypoint}')

        # Convert vector to the perspective of the robot
        new_vector = vector_to_waypoint.rotate(math.radians(-self.orientation))
        # print(new_vector)

        return new_vector


    def get_robot_repulsion_vector(self, ids):

        # Repel other robots
        num_other_robots = 0
        res_vec = Vector2D(0,0)

        # print(Fore.RED + f'ids: {ids}')
        # print(Fore.RED + f'keys: {self.neighbours.keys()}')

        for key, value in self.neighbours.items():
            if key in ids:
                distance = value.range
                lj_force = self.generalized_lennard_jones_repulsion(distance)
                print("LJ force: {}".format(lj_force))
                x = math.cos(math.radians(value.bearing)) * lj_force
                y = math.sin(math.radians(value.bearing)) * lj_force
                rep_vec = Vector2D(x, y)
                print("Vector to repel: {}".format(rep_vec))
                res_vec += rep_vec

            num_other_robots += 1            

        if num_other_robots > 0:
            res_vec /= num_other_robots

        return res_vec


    ### Controllable events ###

    # def _callback_start(self, data):
    #     print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] Start")
    #     self.signal = True
    #     self.input_start = False


    # def _callback_stop(self, data):
    #     print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] Stop")
    #     self.signal = False
    #     self.input_stop = False


    def _callback_message(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] Message")

        # Send a heart-beat message to the other leaders every 10 timesteps
        beat = RelayMsg()
        beat.type = 'H'
        beat.origin = self.id
        beat.origin_team = self.team_id
        beat.time = self.init_step_timer
        beat.follower_num = self.current_follower_count
        beat.task_min_num = self.current_task_min_robot_num

        # # DEBUG
        # if self.id == 17:
        #     if self.init_step_timer == 10:
        #         self.num_robots_to_request = 1
        #     elif self.init_step_timer > 10:
        #         self.num_robots_to_request = 0

        # # DEBUG
        # if self.init_step_timer == 40 and not self.acknowledge_sent and self.id == 16:
        #     self.num_robots_to_send = 1
        #     self.num_robots_remaining_to_send = self.num_robots_to_send
        #     self.team_to_join = 2

        # # DEBUG (Auto request) Ex0. Not for Ex1 or Ex2
        # # Check if it is near the waypoint
        # if self.waypoints.qsize() > 0:
        #     dist = list(self.waypoints.queue)[0].distance_to(self.position)
        #     self.in_task = dist <= Leader.THRES_RANGE
        # else:
        #     self.in_task = True

        # # Try requesting again if requested robots did not arrive
        # if self.init_step_timer > self.all_requests_satisfied_time + 300 and self.current_task_min_robot_num - self.current_follower_count > 0: # TEMP: hard-coded wait value
        #     self.all_requests_satisfied_time = self.init_step_timer
        #     self.all_requests_satisfied = False
        #     self.request_sent = False
        #     print(f'Still insufficient robots. Resetting request process')

        # print(Fore.LIGHTRED_EX + f'{self.in_task}')
        # print(Fore.LIGHTMAGENTA_EX + f'{self.current_task_min_robot_num - self.current_follower_count > 0}')
        # print(Fore.LIGHTYELLOW_EX + f'{self.num_robots_remaining_to_request == 0}')
        # print(Fore.LIGHTCYAN_EX + f'{not self.all_requests_satisfied}')

        # if self.in_task and self.current_task_min_robot_num - self.current_follower_count > 0 and self.num_robots_remaining_to_request == 0 and not self.all_requests_satisfied:
        #     self.num_robots_remaining_to_request = self.current_task_min_robot_num - self.current_follower_count

        # print(Fore.LIGHTRED_EX + f'Remaining to request: {self.num_robots_remaining_to_request}')

        # if self.num_robots_remaining_to_request > 0 and not self.request_sent:

        #     # Request the number of robots needed to the leader with the smallest hop count to itself
        #     closest_teams = []
        #     smallest_hop = 100000
        #     for team, hop in self.hops_dict.items():
        #         if team in self.teams_to_request:
        #             if hop.count < smallest_hop:
        #                 closest_teams.clear()
        #                 closest_teams.append(team)
        #                 smallest_hop = hop.count
        #             elif hop.count == smallest_hop:
        #                 closest_teams.append(team)

        #     closest_teams.sort()

        #     beat.type = 'R'
        #     beat.robot_num = self.num_robots_remaining_to_request
        #     beat.request_to_team = closest_teams[0]
        #     print(f'need: {self.num_robots_remaining_to_request} current_team: {self.current_follower_count}')
        #     print(f'[REQUEST] Requesting {beat.robot_num} robots to team {beat.request_to_team}...')
        #     self.request_sent = True
        #     self.teams_to_request.remove(beat.request_to_team)


        # DEBUG (auto send/reject)
        # # Check if it is near the waypoint
        # if self.waypoints:
        #     dist = list(self.waypoints.queue)[0].distance_to(self.position)
        #     self.in_task = dist <= Leader.THRES_RANGE
        # else:
        #     self.in_task = True

        # # Get number of robots requested
        # # TEMP: Assuming only one team ever requests (behavior analysis)
        # for team, num in self.num_robots_requested.items():
        #     if self.current_follower_count > 0 and not self.acknowledge_sent and self.in_task:
        #         if self.current_follower_count >= num:
        #             self.num_robots_to_send = num
        #         else:
        #             self.num_robots_to_send = self.current_follower_count # Send all available robots
                
        #         self.num_robots_remaining_to_send = self.num_robots_to_send
        #         self.team_to_join = team
        #         print(f'ACCEPT request from team {team}, current: {self.current_follower_count}, num: {self.num_robots_to_send}')
        #     else:
        #         beat.type = 'A'
        #         beat.robot_num = 0
        #         beat.accept_to_team = self.team_to_join
        #         self.acknowledge_sent = True
        #         print(f'REJECT request from team {team}')
        #     break
        # self.num_robots_requested.clear()

        # User signal
        if self.num_robots_to_request > 0:
            beat.type = 'R'
            beat.robot_num = self.num_robots_to_request
            print(Fore.YELLOW + f'[REQUEST] Reqeusting {beat.robot_num} robots...')
            self.log_messages.append(f'[REQUEST] Requesting {beat.robot_num} robots...')
            self.num_robots_to_request = 0

        # Acknowledge message
        # if not self.acknowledge_sent and self.num_robots_to_send > 0:
        #     beat.type = 'A'
        #     beat.robot_num = self.num_robots_to_send
        #     beat.accept_to_team = self.team_to_join
        #     print(Fore.YELLOW + f'[SEND] Sending {beat.robot_num} robots to team {beat.accept_to_team}!')
        #     self.log_messages.append(f'[SEND] Sending {beat.robot_num} robots!')
        #     self.acknowledge_sent = True
        # elif self.num_robots_remaining_to_send == 0:
        #     self.num_robots_to_send = 0
        #     self.acknowledge_sent = False

        self.rmsg_to_resend.append([Leader.SEND_DURATION, beat])
        self.last_sent = self.init_step_timer

        self.beat_sent += 1


    # def _callback_respond(self, data):
    #     print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] Respond")

    #     key_to_delete = []

    #     for key, val in self.accept_ids.items():
    #         response = ConnectionMsg()
    #         if self.team_id < key: # Leader accepts its follower to become a connector if its team ID is smaller than the toher team the follower is trying to join
    #             # Upon receiving a request message, send an accept message to the follower with the smallest ID
    #             response.type = 'A'
    #             response.to = self.accept_ids[key]
    #         else:
    #             response.type = 'N'
    #             response.to = '' # DUMMY
    #             key_to_delete.append(key)

    #         response.origin = self.id
    #         response.to_team = self.team_id
    #         response.other_team = key

    #         self.cmsg_to_resend.append([Leader.SEND_DURATION, response])

    #     # Delete key if it did not send an accept message
    #     for key in key_to_delete:
    #         self.accept_ids.pop(key)

    #     for pair in self.cmsg_to_resend:
    #         print(f'\t(type: {pair[1].type}, for: {pair[1].to})', end='')
    #     print()


    def _callback_exchange(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] Exchange")

        if self.switch_candidate:

            # Signal a follower to switch to the other team
            if not self.decremented:
                if self.init_step_timer - self.robot_last_sent_time > Leader.SEND_ROBOT_DELAY:
                    self.robot_to_switch = self.switch_candidate
                    self.num_robots_remaining_to_send -= 1
                    self.decremented = True
                    self.robot_last_sent_time = self.init_step_timer
                    self.robot_last_sent = self.robot_to_switch

                    # reset
                    self.switch_candidate = ''
                    self.switch_candidate_dist = 100000
        else:
            print(Fore.RED + 'switch_candidate is empty')


    ### Uncontrollable events ###

    def _check_inputMessage(self, data):
        time_to_send = (self.init_step_timer > 0 and self.init_step_timer % 10 == 0)
        return time_to_send


    def _check_inputExchange(self, data):
        exchange_robot = self.num_robots_remaining_to_send > 0 and self.switch_candidate
        if exchange_robot:
            return True
        return False


class Worker(Robot):

    # Class parameters

    # Flocking params
    TARGET_DISTANCE = 0.08
    GAIN = 100
    EXPONENT = 2

    SEPARATION_THRES = 0.3
    JOINING_THRES = 0.27

    # Weights using for the motion in each role
    FOLLOWER_ATTRACTION = 1
    FOLLOWER_REPULSION = 0.01
    FOLLOWER_OBSTACLE = 1

    CONNECTOR_ATTRACTION_TO_CONNECTOR = 1
    CONNECTOR_ATTRACTION_TO_TEAM = 1
    CONNECTOR_REPULSION = 0.01
    CONNECTOR_OBSTACLE = 1
    CONNECTOR_TARGET_DISTANCE = 0

    TRAVELER_ATTRACTION = 1
    TRAVELER_REPULSION = 0.01
    TRAVELER_OBSTACLE = 1
    TRAVELER_JOINING_THRES = 1

    SEND_DURATION = 4
    SEND_RESPOND_DURATION = 6
    WAIT_REQUEST_DURATION = 8

    SCT_PATH = 'dotbot/examples/network_formation/models/worker_nop.yaml'

    @classmethod
    def load_params(cls, config):

        for param in config[0]:

            if param.tag =='team_flocking':
                cls.TARGET_DISTANCE = float(param.get('target_distance'))
                cls.GAIN = float(param.get('gain'))
                cls.EXPONENT = float(param.get('exponent'))

            elif param.tag == 'team_distance':
                cls.SEPARATION_THRES = float(param.get('separation_threshold'))
                cls.JOINING_THRES = float(param.get('joining_threshold'))

            elif param.tag == 'follower':
                cls.FOLLOWER_ATTRACTION = float(param.get('attract'))
                cls.FOLLOWER_REPULSION = float(param.get('repulse'))
                cls.FOLLOWER_OBSTACLE = float(param.get('obstacle'))

            elif param.tag == 'connector':
                cls.CONNECTOR_ATTRACTION_TO_CONNECTOR = float(param.get('attract_to_connector'))
                cls.CONNECTOR_ATTRACTION_TO_TEAM = float(param.get('attract_to_team'))
                cls.CONNECTOR_REPULSION = float(param.get('repulse'))
                cls.CONNECTOR_OBSTACLE = float(param.get('obstacle'))
                cls.CONNECTOR_TARGET_DISTANCE = float(param.get('target_distance'))

            elif param.tag == 'traveler':
                cls.TRAVELER_ATTRACTION = float(param.get('attract'))
                cls.TRAVELER_REPULSION = float(param.get('repulse'))
                cls.TRAVELER_OBSTACLE = float(param.get('obstacle'))
                cls.TRAVELER_JOINING_THRES = float(param.get('joining_threshold'))

            elif param.tag == 'timeout':
                cls.SEND_DURATION = float(param.get('send_message'))
                cls.SEND_RESPOND_DURATION = float(param.get('send_respond'))
                cls.WAIT_REQUEST_DURATION = float(param.get('wait_request'))

            elif param.tag == 'SCT':
                cls.SCT_PATH = param.get('path')


    def __init__(self, robot_id, yaml='', team_id=1):
        super().__init__(robot_id, yaml, team_id)

        self.id = self.id

        self.left = 0
        self.right = 0

        ### Framework related (worker) ###

        self.current_state = RobotState.FOLLOWER

        self.current_move_type = MoveType.ADJUST

        # Add SCT callback functions
        self.sct = SCTPub(Worker.SCT_PATH)
        self.add_callbacks()

        # Team info
        self.leader_msg = Message()
        self.hop_count_to_leader = 100000
        # self.leader_signal = 0 # int, 0-stop working on task, 1-work on task
        self.robot_to_switch = ''
        self.team_to_join = None

        self.performing_task = True # Set default to true for now

        self.role_switch_conditions_per_team = defaultdict(RoleSwitchConditions)
        self.received_request = False
        self.received_accept = False
        self.received_reject = False

        self.current_request = ConnectionMsg()
        self.current_accept = ConnectionMsg()
        self.request_timer = 0 # int, Remaining timesteps to wait since a request was made

        self.connection_candidates = {}

        self.robots_to_accept = {} # List of robots to accept as connectors in the current timestep

        self.hops_copy = {} # The hop count info of the connector this robot will connect with
        self.prev_hops = {} # The previous connection if hop is still new (i.e. connection in hopsDict still appears in cmsgToResend)

        self.nearby_teams = {} # Teams that are within the safety range and the average distance to the team

        self.last_beat = {} # RelayMsg indexed with team_id and attached with a boolean to determine whether a new message was received in this timestep [RelayMsg, char]

        self.nearLF = False # Whether a traveler is near the leader or follower of the team it is trying to join

        # Network change related variables
        self.tail_switch_timer = 0 # Timer that decrements. Prevent connectors from letting a neighbor become the new tail connector when it has just become one
        self.connector_switch_timer = 0 # Timer that decrements. Prevent connectors from triggering network change when it has just become a connector
        self.network_change_timer = 0 # Timer that decrements. Prevent connectors from triggering network change when it has just applied another network change

        self.network_changes = {} # key: team, val: pair<prevRobotID, newRobotMessage>
        self.notify_sent = False # flag to indicate whether notifyNM was triggered
        self.nmsgs_received = [] # Message
        self.nmsg_to_send = [] # NetworkChangeMsg
        self.nmsg_to_resend = [] # pair (time, NetworkChangeMsg)
        self.new_connections = [] # pair (time, robot_id). robot ids attached with a timer. Any robot ids here will be ignored even if the other connector doesn't have its id in their hopsDict


    # Register callback functions to the generator player
    def add_callbacks(self):

        # Automatic addition of callbacks
        # 1. Get list of events and list specifying whether an event is controllable or not.
        # 2. For each event, check controllable or not and add callback.

        events, controllability_list = self.sct.get_events()

        for event, index in events.items():
            is_controllable = controllability_list[index]
            stripped_name = event.split('EV_', 1)[1]    # Strip preceding string 'EV_'

            if is_controllable: # Add controllable event
                func_name = '_callback_{0}'.format(stripped_name)
                func = getattr(self, func_name)
                self.sct.add_callback(self.sct.EV[event], func, None, None)
            else:   # Add uncontrollable event
                func_name = '_check_{0}'.format(stripped_name)
                func = getattr(self, func_name)
                self.sct.add_callback(self.sct.EV[event], None, func, None)


    # Flocking repulsion
    def generalized_lennard_jones_repulsion(self, distance):
        f_norm_dist_exp = pow(Worker.TARGET_DISTANCE / distance, Worker.EXPONENT)
        return -Worker.GAIN / distance * (f_norm_dist_exp * f_norm_dist_exp)


    def control_step(self):
        
        self.init_step_timer += 1

        # DEBUG
        print(Fore.LIGHTCYAN_EX + f'--- Robot {self.id} ---')
        print(Fore.LIGHTCYAN_EX + f'state: {self.current_state}, id: {self.id}, team id: {self.team_id}, move type: {self.current_move_type}, performing task: {self.performing_task}')

        self.reset_variables()

        # TEMP
        print(Fore.LIGHTBLUE_EX + f'hops_dict: ', end='')
        for id, hop in self.hops_dict.items():
            print(Fore.LIGHTBLUE_EX + f'({id}, id:{hop.id}, hop:{hop.count}, resend:{hop.resend_count})', end='')
        print()
        ###

        # Receive messages
        self.get_messages()

        # Update sensor readings
        self.update()

        # Run SCT controller
        if self.init_step_timer > 4:
            self.sct.run_step()
        print(f'Supervisors: {self.sct.sup_current_state}')

        # Create message
        msg = Message()
        msg.state = self.current_state
        msg.id = self.id
        msg.team_id = self.team_id
        msg.robot_to_switch = self.robot_to_switch
        msg.team_to_join = self.team_to_join

        # Update sensor readings and process messages received
        if self.current_state == RobotState.FOLLOWER:
            # Set LED colors
            self.led_colour = (0,255,0) # green

            # Set leader signal
            # msg.leader_signal = self.leader_signal

            # Set team hop count
            msg.team_hop_count = self.hop_count_to_leader

            # Set team shared message
            msg.tmsg = deepcopy(self.team_shared_msg_dict)

            # Set network hop counts
            msg.hops = deepcopy(self.hops_dict)
            for id in msg.hops:
                msg.hops[id].resend_count += 1

        elif self.current_state == RobotState.CONNECTOR:
            # Set LED colors
            self.led_colour = (0,0,255) # blue
            
            # Set network hop counts
            msg.hops = deepcopy(self.hops_dict)

            for pair in self.cmsg_to_resend:
                if pair[1].type == 'A':
                    hop = HopMsg()
                    hop.id = pair[1].to
                    hop.count = 1
                    self.prev_hops[pair[1].to_team] = hop

        elif self.current_state == RobotState.TRAVELER:
            # Set LED colors
            if self.init_step_timer % 2 == 1:
                self.led_colour = (255,255,0) # yellow
            else:
                self.led_colour = (0,0,0) # off

        # Set movement type
        if self.current_move_type == MoveType.FLOCK:
            self.left, self.right = self.flock()
            # TEMP
            # self.left, self.right = 0, 0
        elif self.current_move_type == MoveType.ADJUST:
            self.left, self.right = self.adjust_position()
            # TEMP
            # self.left, self.right = 0, 0
        elif self.current_move_type == MoveType.TRAVEL:
            self.left, self.right = self.travel()

        # Other message contents

        # Decrement timer for newConnections
        self.new_connections = [x for x in self.new_connections if x[0] > 0] # Remove messages that have timed out
        for resend_msg in self.new_connections:
            resend_msg[0] -= 1 # Decrement timer

        # Network Change Message
        self.nmsg_to_resend = [x for x in self.nmsg_to_resend if x[0] > 0] # Remove messages that have timed out
        for resend_msg in self.nmsg_to_resend:
            self.nmsg_to_send.append(resend_msg[1])
            resend_msg[0] -= 1 # Decrement timer

        msg.nmsg = deepcopy(self.nmsg_to_send)

        # Connection Message
        if self.received_accept:
            self.cmsg_to_resend.clear()

        # print(self.cmsg_to_resend)
        self.cmsg_to_resend = [x for x in self.cmsg_to_resend if x[0] > 0] # Remove messages that have timed out
        # self.cmsg_to_resend = [x for x in self.cmsg_to_resend if self.received_accept or self.received_reject] # TEMP: stop resending, response has been received
        
        for resend_msg in self.cmsg_to_resend:
            self.cmsg_to_send.append(resend_msg[1])
            resend_msg[0] -= 1 # Decrement timer

        msg.cmsg = deepcopy(self.cmsg_to_send)

        msg.nearby_teams = deepcopy(self.nearby_teams)

        # Relay Message
        self.rmsg_to_resend = [x for x in self.rmsg_to_resend if x[0] > 0] # Remove messages that have timed out
        for resend_msg in self.rmsg_to_resend:
            self.rmsg_to_send.append(resend_msg[1])
            resend_msg[0] -= 1 # Decrement timer

        msg.rmsg = deepcopy(self.rmsg_to_send)

        # Set ID of all connections to msg
        msg.connections = list(self.neighbours.keys())
        # print(f'connections: {msg.connections}')

        # Set message to send in this timestep
        self.msg = msg

        # Delete old messages
        self.messages.clear()

        # TEMP
        print(Fore.LIGHTBLUE_EX + f'hops_dict: ', end='')
        for id, hop in self.hops_dict.items():
            print(Fore.LIGHTBLUE_EX + f'({id}, id:{hop.id}, hop:{hop.count}, resend:{hop.resend_count})', end='')
        print()
        ###

        print()


    def reset_variables(self):
        self.leader_msg = Message()
        self.team_msgs.clear()
        self.connector_msgs.clear()
        self.other_leader_msgs.clear()
        self.other_team_msgs.clear()
        self.traveler_msgs.clear()

        self.cmsg_to_send.clear()
        self.rmsg_to_send.clear()
        self.nmsg_to_send.clear()

        self.prev_hops.clear()

        self.team_shared_msg_dict.clear()
        self.connection_candidates.clear()
        self.role_switch_conditions_per_team.clear()

        # self.leader_signal = 0
        self.hop_count_to_leader = 100000

        self.received_request = False
        self.received_accept = False
        self.received_reject = False

        for key, val in self.last_beat.items():
            val[1] = 'N' # Reset received flad to N (None)

        self.robots_to_accept.clear()
        self.nearby_teams.clear()

        if not self.notify_sent:
            self.network_changes.clear()
        self.nmsgs_received.clear()

        self.nearLF = False

    
    def get_messages(self):
        # print(f'messages: {self.messages}')

        for id, msg in self.messages.items():

            # Set vector pointing at neighbor (with respect to its own local frame)
            if str(id) in self.neighbours:
                msg.direction = get_vector(self.neighbours[str(id)])

            if msg.state == RobotState.LEADER:
                if id in leader_id.values() and msg.team_id == self.team_id:
                    self.leader_msg = msg
                else:
                    self.other_leader_msgs.append(msg)
            elif msg.state == RobotState.FOLLOWER:
                if msg.team_id == self.team_id:
                    self.team_msgs.append(msg)
                else:
                    self.other_team_msgs.append(msg)
            elif msg.state == RobotState.CONNECTOR:
                self.connector_msgs.append(msg)
            elif msg.state == RobotState.TRAVELER:
                self.traveler_msgs.append(msg)

        # DEBUG
        # for id, msg in self.messages.items():
        #     print(f"id: {id}, team id: {msg.team_id}, leader signal: {msg.leader_signal}, team hop count: {msg.team_hop_count}, to_switch: {msg.robot_to_switch}, to_team: {msg.team_to_join}")

        #     for team_id, hop in msg.hops.items():
        #         print(f'\t[hop] team_id: {team_id}, id: {hop.id}, count: {hop.count}, resend_count: {hop.resend_count}')
        #     for team_id, tmsg in msg.tmsg.items():
        #         print(f'\t[tmsg] team_id: {team_id}, dist: {tmsg.dist}, up: {tmsg.connector_id_upstream}, down: {tmsg.connector_id_downstream}')
        #     for cmsg in msg.cmsg:
        #         print(f'\t[cmsg] type: {cmsg.type}, from: {cmsg.origin}, to: {cmsg.to}, to_team: {cmsg.to_team}')
        #     for rmsg in msg.rmsg:
        #         print(f'\t[rmsg] type: {rmsg.type}, from: {rmsg.origin}, time: {rmsg.time}, robot_num: {rmsg.robot_num}')


    def update(self):

        if self.current_state == RobotState.FOLLOWER:

            self.update_hop_counts_follower()

            self.get_leader_info()

            self.set_connector_to_relay()
            self.set_distance_to_relay()

            self.connection_candidates = self.get_closest_non_team()

            print(f'connection_candidates: {self.connection_candidates}')

            if self.connection_candidates:
                self.switch_to_connector_conditions(self.connection_candidates)

            # Check whether it has received an accept message
            if self.current_request.type == 'R':
                self.check_accept()

                # Decrement timer
                self.request_timer -= 1
                print(f'request_timer: {self.request_timer}')

                # Check whether an accept message was not received before the timeout
                if self.request_timer <= 0:
                    self.received_reject = True

            self.set_cmsgs_to_relay()
            self.set_leader_msg_to_relay(self.current_state)

            # print(self.hops_dict)
            # print(self.team_shared_msg_dict)

        elif self.current_state == RobotState.CONNECTOR:

            if self.tail_switch_timer > 0:
                self.tail_switch_timer -= 1
            if self.connector_switch_timer > 0:
                self.connector_switch_timer -= 1
            if self.network_change_timer > 0:
                self.network_change_timer -= 1

            self.check_requests()

            self.update_hop_counts_connector()

            self.store_network_modification()
            self.check_network_modification()

            # # Find the team_ids of followers that are within its safety range
            # nearby_msgs = self.other_team_msgs + self.other_leader_msgs

            # for msg in nearby_msgs:
            #     dist = get_distance_to_robot(self.neighbours[str(msg.id)])
            #     if not msg.team_id in self.nearby_teams:
            #         if dist < Worker.JOINING_THRES:
            #             self.nearby_teams.add(msg.team_id)

            self.set_leader_msg_to_relay(self.current_state)

            self.switch_to_follower_conditions()

        elif self.current_state == RobotState.TRAVELER:

            combined_msgs = self.other_team_msgs + self.other_leader_msgs

            # Check whether it has reached the other team
            for msg in combined_msgs:
                if msg.team_id == self.team_to_join:
                    if abs(msg.direction) < Worker.TRAVELER_JOINING_THRES and msg.team_hop_count <= 1:
                        self.nearLF = True
                        break


    def get_leader_info(self):
        if not self.leader_msg.empty(): # If leader is in range
            self.hop_count_to_leader = 1
            # self.leader_signal = self.leader_msg.leader_signal

            # Comment out below for Ex2 with 0 second delay
            if self.robot_to_switch != self.id:
                self.robot_to_switch = self.leader_msg.robot_to_switch
                self.team_to_join = self.leader_msg.team_to_join

        else: # If leader is NOT in range

            min_count = 100000 # Large number

            # Find the smallest hop count among team members
            for msg in self.team_msgs:
                if msg.team_hop_count < min_count:
                    min_count = msg.team_hop_count
            
            # Record its own hop count
            if min_count < 100000:
                self.hop_count_to_leader = min_count + 1

            for msg in self.team_msgs:
                if msg.team_hop_count < min_count:
                    # self.leader_signal = msg.leader_signal
                    self.robot_to_switch = msg.robot_to_switch
                    self.team_to_join = msg.team_to_join
                    break

        print(Fore.LIGHTBLUE_EX + f"Hop count to leader: {self.hop_count_to_leader}")
    

    def get_closest_non_team(self):
        closest_robots = {}

        # Return a connection candidate for each team
        for id in self.team_shared_msg_dict:

            # Check for the robot that this robot can connect
            min_dist = 100000
            candidate_msgs = []
            closest_robot = None # Message

            # # Prioritize connectors over other team members
            # if self.connector_msgs:
            #     candidate_msgs = deepcopy(self.connector_msgs)
            # else:
            #     candidate_msgs = self.other_leader_msgs + self.other_team_msgs
            candidate_msgs = deepcopy(self.connector_msgs)

            # Find the closest non team robot from the matching team
            for msg in candidate_msgs:
                dist = abs(msg.direction)

                if dist < min_dist:
                    if (msg.state == RobotState.FOLLOWER and msg.team_id == id) or \
                       (msg.state == RobotState.CONNECTOR and self.team_id in msg.hops and msg.hops[self.team_id].count == 1):

                        min_dist = dist
                        closest_robot = msg

            if closest_robot:
                closest_robots[id] = closest_robot

        return closest_robots


    def switch_to_connector_conditions(self, msgs):
        """Find whether this robot is the closest to any of the connection candidates among its teammates"""
        
        for id, msg in msgs.items():
            is_closest = True
            my_dist = abs(msg.direction)

            # condC1: shortest distance to the team
            
            if my_dist >= Worker.SEPARATION_THRES:
                self.role_switch_conditions_per_team[id].c1 = True
            else:
                self.role_switch_conditions_per_team[id].c1 = False

            # condC2: whether it is the closest among teammates

            # If the team has identified the nex connector to connect to, check if it is the same
            if self.team_shared_msg_dict[id].connector_id_downstream:
                if msg.id != self.team_shared_msg_dict[id].connector_id_downstream:
                    self.role_switch_conditions_per_team[id].c2 = False
                    continue

            # Check whether it is the closest to the candidate among other followers in the team that sees it (condC2)
            for team_msg in self.team_msgs:
                connections = team_msg.connections.copy()

                # Check if the team robot has seen the non-team robot
                if not msg.id in connections:
                    
                    # Check the distance between its candidate and nearby team robots
                    candidate_vec = get_vector(self.neighbours[str(msg.id)])
                    teammate_vec = get_vector(self.neighbours[str(team_msg.id)])
                    dist = abs(candidate_vec - teammate_vec)

                    if dist + 0.1 < my_dist: # TEMP: Fixed extra buffer
                        is_closest = False # Not the closest to the candidate robot
                        self.role_switch_conditions_per_team[id].c2 = False

            if is_closest:
                self.role_switch_conditions_per_team[id].c2 = True # It is the closest

            # condC3: whether the distance between the leader and connection candidate exceeds the separation threshold

            if self.leader_msg.empty():
                self.role_switch_conditions_per_team[id].c3 = True # true, if leader is not in range
            else:
                candidate_vec = get_vector(self.neighbours[str(msg.id)])
                leader_vec = get_vector(self.neighbours[str(self.leader_msg.id)])
                dist = abs(candidate_vec - leader_vec)

                if dist > Worker.SEPARATION_THRES:
                    self.role_switch_conditions_per_team[id].c3 = True
                else:
                    self.role_switch_conditions_per_team[id].c3 = False # If leader is clsoe, no need to become a connector

        # print(self.role_switch_conditions_per_team)
        for key, val in self.role_switch_conditions_per_team.items():
            print(f'id: {key}, c1: {val.c1}, c2: {val.c2}, c3: {val.c3}')


    def switch_to_follower_conditions(self):
        
        # Check if it is allowed to switch to a follower

        # Extract adjacent connector IDs to check
        robot_ids = set()
        for id, hop in self.hops_dict.items():
            if hop.id:
                robot_ids.add(hop.id)

        for key, hop in self.hops_dict.items():

            # condF1: Is it a tail connector for this team?

            if hop.count == 1:
                self.role_switch_conditions_per_team[key].f1 = True

            # condF2: Are the successors all closer to the team than itself?
            # If it is the only connector, are the teams close to each other?

            if robot_ids:
                # If it is connected to at least one connector

                needed_as_connector = False

                # Find the other connector message
                for id in robot_ids:
                    for msg in self.connector_msgs:
                        if msg.id == id:

                            # If it appears in adjacent connector's hops_dict, check if the team is also close.
                            # If adjacent connector is near team, it is not needed as connector for this team.

                            is_near_team = False

                            # Are the other hop counts smaller?
                            has_smaller_hop_counts = False
                            is_hop_smaller_map = {}

                            for other_id, other_hop in msg.hops.items():
                                if other_hop.id == self.id and other_id == key:

                                    for team, dist in msg.nearby_teams.items():
                                        if key == team and dist < Worker.JOINING_THRES:
                                            is_near_team = True
                                            break

                                # Check if any other hop count is larger or not
                                if other_id != key:

                                    if other_hop.count < self.hops_dict[other_id].count: # TODO: Check if key exists in dict
                                        is_hop_smaller_map[other_id] = True
                                    else:
                                        is_hop_smaller_map[other_id] = False

                            # Check if all hop counts are smaller
                            larger_hop_exists = False
                            for other_id, other_hop in is_hop_smaller_map.items():
                                if not other_hop:
                                    larger_hop_exists = True
                            
                            if not larger_hop_exists:
                                has_smaller_hop_counts = True

                            if not is_near_team or not has_smaller_hop_counts:
                                needed_as_connector = True

                            self.role_switch_conditions_per_team[key].f2 = not needed_as_connector # If needed, set false

                            break
            
            else:
                pass

                # is_only_connector = True

                # for id, hop in self.hops_dict.items():
                #     if hop.count != 1:
                #         is_only_connector = False

                # # It is the only connector between teams

                # if is_only_connector:

                #     # Check the shortest distance between the followers of the two teams
                #     # Followers that are visible to this robot

                #     # Split other_team_msgs

                #     split_team_msgs = {}
                #     for msg in self.other_leader_msgs:
                #         if not msg.team_id in split_team_msgs:
                #             # If key doesn't exist, create new entry
                #             split_team_msgs[msg.team_id] = []
                #         split_team_msgs[msg.team_id].append(msg)

                #     for msg in self.other_team_msgs:
                #         if not msg.team_id in split_team_msgs:
                #             # If key doesn't exist, create new entry
                #             split_team_msgs[msg.team_id] = []
                #         split_team_msgs[msg.team_id].append(msg)

                #     # Find the shortest distance between the two teams it connects to (O^2)

                #     print(split_team_msgs)

                #     # TEMP: Hardcoded team numbers
                #     team1 = 1
                #     team2 = 2

                #     min_dist = 100000

                #     if team1 in split_team_msgs:
                #         for msg1 in split_team_msgs[team1]:
                #             for msg2 in split_team_msgs[team2]:
                #                 diff = get_vector(self.neighbours[str(msg1.id)]) - get_vector(self.neighbours[str(msg2.id)])
                #                 dist = abs(diff)

                #                 if dist < min_dist:
                #                     min_dist = dist

                #     # The followers of the two teams are close by. Then this robot is not needed.
                #     if min_dist < Worker.SEPARATION_THRES - 0.1:
                #         self.role_switch_conditions_per_team[key].f2 = True

        for key, val in self.role_switch_conditions_per_team.items():
            print(f'id: {key}, f1: {val.f1}, f2: {val.f2}')


    def check_accept(self):
        # print('check_accept()')

        # # Request sent to leader
        # if self.current_request.to == leader_id[self.team_id]:

        #     combined_team_msgs = self.team_msgs.copy()
        #     if not self.leader_msg.empty():
        #         combined_team_msgs.append(self.leader_msg)

        #     for msg in combined_team_msgs:
        #         for cmsg in msg.cmsg:
        #             if cmsg.type == 'A':

        #                 if cmsg.to == self.id:              # Request approved for this worker
        #                     self.received_accept = True
        #                     self.current_accept = cmsg
        #                 else:                               # Request approved for another worker
        #                     self.received_reject = True

        #             elif cmsg.type == 'N':
        #                 self.received_reject = True

        # Request sent to connector
        for msg in self.connector_msgs:
            for cmsg in msg.cmsg:
                if cmsg.type == 'A':

                    # Check the connector matches its original request and is directed to its current team
                    if cmsg.origin == self.current_request.to and cmsg.to_team == self.team_id:
                        if cmsg.to == self.id:          # Request approved for this robot
                            self.received_accept = True
                            self.current_accept = cmsg
                            self.hops_copy = deepcopy(msg.hops)
                        else:                           # Request approved for another robot
                            self.received_reject = True


    def check_requests(self):

        # Check the shortest distance to each team
        team_distances = {}

        for id, hop in self.hops_dict.items():
            for msg in self.other_team_msgs:
                if msg.team_id == id and not hop.id: # If hop count == 1 for a given team

                    dist = abs(msg.direction)

                    if not id in team_distances:
                        team_distances[id] = dist
                    else:
                        if dist < team_distances[id]:
                            team_distances[id] = dist

        # Check all requests sent to itself and choose one to respond to each team
        for msg in self.other_team_msgs:
            for cmsg in msg.cmsg:
                if cmsg.to == self.id and cmsg.type == 'R':
                    print(f'request from: {msg.id}')
                    self.received_request = True

                    # Accept if it does not have a fixed connector (ID field is empty)
                    if not self.hops_dict[msg.team_id].id:

                        # Accept if the distance to all robots from that team is far
                        if team_distances[msg.team_id] > Worker.SEPARATION_THRES:

                            # Accept first request seen for a team
                            if not msg.team_id in self.robots_to_accept:
                                self.robots_to_accept[msg.team_id] = msg
                                continue

                            current_dist = abs(self.robots_to_accept[msg.team_id].direction)
                            new_dist = abs(msg.direction)

                            # Send an accept message to the closest follower
                            if new_dist < current_dist:
                                self.robots_to_accept[msg.team_id] = msg

        print(f'self.robots_to_accept: {self.robots_to_accept}')


    def set_cmsgs_to_relay(self):
        
        # Messages from the leader and other followers in the same team
        combined_msgs = self.team_msgs + [self.leader_msg]

        # Relay any number of request or accept messages

        for msg in combined_msgs:
            for cmsg in msg.cmsg:

                # Relay Request messages received from robots with greater hop count
                if cmsg.type == 'R' and msg.team_hop_count > self.hop_count_to_leader:
                    if self.current_request.type != 'R': # Check if it is currently not requesting
                        self.cmsg_to_send.append(cmsg)
                
                # Relay Accept messages received from robots with smaller hop count
                if cmsg.type == 'A' and msg.team_hop_count < self.hop_count_to_leader:
                    self.cmsg_to_send.append(cmsg)


    def set_leader_msg_to_relay(self, state):
        
        if state == RobotState.FOLLOWER:

            # Add connectors and leaders/followers from other teams to check for inward message relay
            inward_msgs = self.other_leader_msgs + self.other_team_msgs + self.connector_msgs

            # Add the leader to check for outward message to relay
            outward_msgs = []
            if not self.leader_msg.empty():
                outward_msgs.append(self.leader_msg)

            # Split messages from team followers into two groups
            for msg in self.team_msgs:
                if msg.rmsg:
                    if msg.team_hop_count > self.hop_count_to_leader:
                        inward_msgs.append(msg)
                    elif msg.team_hop_count < self.hop_count_to_leader:
                        outward_msgs.append(msg)

            # print(f'INWARD: {inward_msgs}')
            # print(f'OUTWARD: {outward_msgs}')

            # For inward message, find all that's not in resend
            for msg in inward_msgs:
                for relay_msg in msg.rmsg:
                    received_team_id = relay_msg.origin_team
                    if received_team_id != self.team_id:
                        if not received_team_id in self.last_beat: # If it's the first time receiving, add it to last_beat received
                            if msg.state == RobotState.LEADER:
                                relay_msg.first_follower = self.id
                                relay_msg.first_follower_dist = abs(msg.direction)
                                self.last_beat[received_team_id] = [relay_msg, 'L']
                            elif msg.team_id != self.team_id:
                                relay_msg.first_follower = self.id
                                relay_msg.first_follower_dist = abs(msg.direction)
                                self.last_beat[received_team_id] = [relay_msg, 'W']
                            else:
                                self.last_beat[received_team_id] = [relay_msg, 'W']
                        else:
                            if relay_msg.time > self.last_beat[received_team_id][0].time: # Else update it only if the timestep is newer
                                if msg.state == RobotState.LEADER:
                                    relay_msg.first_follower = self.id
                                    relay_msg.first_follower_dist = abs(msg.direction)
                                    self.last_beat[received_team_id] = [relay_msg, 'L']
                                elif msg.team_id != self.team_id:
                                    relay_msg.first_follower = self.id
                                    relay_msg.first_follower_dist = abs(msg.direction)
                                    self.last_beat[received_team_id] = [relay_msg, 'W']
                                else:
                                    self.last_beat[received_team_id] = [relay_msg, 'W']

            # For outward message, only find one
            for msg in outward_msgs:
                for relay_msg in msg.rmsg:
                    if relay_msg.origin_team == self.team_id:
                        if not self.team_id in self.last_beat: # If it's the first time receiving, add it to last_beat received
                            if msg.state == RobotState.LEADER:
                                self.last_beat[self.team_id] = [relay_msg, 'L']
                            else:
                                self.last_beat[self.team_id] = [relay_msg, 'W']
                        else:
                            if relay_msg.time > self.last_beat[self.team_id][0].time: # Else update it only if the timestep is newer
                                if msg.state == RobotState.LEADER:
                                    self.last_beat[self.team_id] = [relay_msg, 'L']
                                else:
                                    self.last_beat[self.team_id] = [relay_msg, 'W']

        elif state == RobotState.CONNECTOR:
            
            # Check whether new relay_msg is received
            for id, hop in self.hops_dict.items():
                # Check the team
                if hop.count == 1 or hop.count == 2:

                    combined_msgs = self.other_leader_msgs + self.other_team_msgs

                    for msg in combined_msgs:
                        if msg.team_id == id:
                            for relay_msg in msg.rmsg:
                                if relay_msg.origin_team == id:
                                    if not id in self.last_beat: # If it's the first time receiving, add it to last_beat received
                                        if msg.state == RobotState.LEADER:
                                            self.last_beat[id] = [relay_msg, 'L']
                                        else:
                                            self.last_beat[id] = [relay_msg, 'W']
                                    else:
                                        if relay_msg.time > self.last_beat[id][0].time: # Else update it only if the timestep is newer
                                            if msg.state == RobotState.LEADER:
                                                self.last_beat[id] = [relay_msg, 'L']
                                            else:
                                                self.last_beat[id] = [relay_msg, 'W']

                # Check the connectors
                if hop.count > 1:
                    for msg in self.connector_msgs:
                        if hop.id == msg.id:
                            for relay_msg in msg.rmsg:
                                if relay_msg.origin_team == id:
                                    if not id in self.last_beat: # If it's the first time receiving, add it to last_beat received
                                        self.last_beat[id] = [relay_msg, 'W']
                                    else:
                                        if relay_msg.time > self.last_beat[id][0].time: # Else update it only if the timestep is newer
                                            self.last_beat[id] = [relay_msg, 'W']


    def set_connector_to_relay(self):

        # Get all visible teams from connectors and followers
        combined_msgs = self.connector_msgs + self.team_msgs

        team_ids = set() # set of team ids
        for msg in combined_msgs:
            for id, hop in msg.hops.items():
                if id != 0: # TEMP solution from accidentally creating a new entry for team 0
                    if msg.state == RobotState.CONNECTOR:
                        team_ids.add(id)
                    if msg.state == RobotState.FOLLOWER:
                        team_ids.add(id)

        if self.team_id in team_ids:
            team_ids.remove(self.team_id) # Delete its own team id

        # Add new entries to team_shared_msg_dict if it doesn't exist
        for id in team_ids:
            if not id in self.team_shared_msg_dict:
                self.team_shared_msg_dict[id] = TeamSharedMsg()

        # Update connector to relay upstream to the leader
        for other_team_id, team_msg in self.team_shared_msg_dict.items():
            
            # Check if it detects the tail connector directly
            connector_nearby = False

            min_hop_count = 100000
            min_hop_count_robot = ''

            for msg in self.connector_msgs:
                # Find connector with the smallest hop count to the team and record it
                for id, hop in msg.hops.items():
                    if self.team_id in msg.hops:
                        if id == other_team_id and msg.hops[self.team_id].count == 1:
                            if min_hop_count_robot == '' or hop.count < min_hop_count:
                                min_hop_count = hop.count
                                min_hop_count_robot = msg.id
                                connector_nearby = True

            if connector_nearby:
                self.team_shared_msg_dict[other_team_id].connector_id_upstream = min_hop_count_robot # Update ID to send upstream to team
            else:
                if self.team_msgs:
                    previous_seen = False
                    new_value = False

                    for msg in self.team_msgs:
                        if msg.team_hop_count > self.hop_count_to_leader: # Only check if it is further away from the leader
                            new_info_found = False
                            for id, follower_team_msg in msg.tmsg.items():
                                if id == other_team_id:

                                    if follower_team_msg.connector_id_upstream == self.team_shared_msg_dict[other_team_id].connector_id_upstream:

                                        previous_seen = True # Received the same connector as before. Confirms current data is correct

                                    elif follower_team_msg.connector_id_upstream:

                                        # Update connector info
                                        self.team_shared_msg_dict[other_team_id].connector_id_upstream = follower_team_msg.connector_id_upstream
                                        new_value = True
                                        new_info_found = True
                                        break

                            if new_info_found:
                                break

                    if not previous_seen and not new_value: # If previous info not received and no new info, reset upstream
                        self.team_shared_msg_dict[other_team_id].connector_id_upstream = ''

                else:
                    # Found and received no info about this tea, so reset upstream
                    self.team_shared_msg_dict[other_team_id].connector_id_upstream = ''

            # Update connector to relay downstream
            combined_msgs = deepcopy(self.team_msgs)
            if not self.leader_msg.empty():
                combined_msgs.append(self.leader_msg)

            if combined_msgs:
                for msg in combined_msgs:
                    if msg.team_hop_count < self.hop_count_to_leader:
                        new_info_found = False
                        for id, follower_team_msg in msg.tmsg.items():
                            if id == other_team_id:
                                self.team_shared_msg_dict[other_team_id].connector_id_downstream = follower_team_msg.connector_id_downstream
                                new_info_found = True
                                break

                        if new_info_found:
                            break
            else:
                # Received no info about this team, so reset downstream
                self.team_shared_msg_dict[other_team_id].connector_id_downstream = ''

        # print(f'self.team_shared_msg_dict: {self.team_shared_msg_dict}')


    def set_distance_to_relay(self):
        """Calculate the shortest distance to another team to share to the rest of the team"""

        for id, tmsg in self.team_shared_msg_dict.items():
            if not tmsg.connector_id_upstream: # If empty
                min_dist = 100000

                # Find the shortest distance to a team (if directly visible)
                for msg in self.other_team_msgs:
                    if msg.team_id == id:
                        dist = abs(msg.direction)

                        if dist < min_dist:
                            min_dist = dist
        
                # Find the shortest distance to a team (received from team)
                for msg in self.team_msgs:
                    if id in msg.tmsg: # If dict contains the team ID
                        if msg.team_hop_count > self.hop_count_to_leader:
                            if msg.tmsg[id].dist < min_dist:
                                min_dist = msg.tmsg[id].dist

                tmsg.dist = min_dist


    def update_hop_counts_connector(self):
        
        # Add every other visible team to hop map
        for msg in self.other_team_msgs:

            # Add hop count entry if not yet registered
            if not msg.team_id in self.hops_dict:
                hop = HopMsg()
                hop.count = 1
                self.hops_dict[msg.team_id] = hop

        # Extract neighboring connector IDs to check
        robot_ids = set()
        for id, hop in self.hops_dict.items():
            if hop.id:
                robot_ids.add(hop.id)
        # print(f'robot_ids: {robot_ids}')

        # Extract messages from connectors that have the IDs found previously
        robot_messages = {}
        for msg in self.connector_msgs:

            if not robot_ids:
                break

            # Find the next connector
            if msg.id in robot_ids:
                robot_messages[msg.id] = msg
                robot_ids.remove(msg.id)

        # # Become a new tail connector if necessary

        # # If the neighbor is still a connector and it is poining to this robot now, become a new connector
        # if self.network_change_timer == 0 and self.tail_switch_timer == 0:
        #     for team, hop in self.hops_dict.items():
        #         if hop.id in robot_messages:
        #             if robot_messages[hop.id].hops[team].id == self.id and hop.count == 2: # Check that its own previously known info about the neighbor is hop=2 (the neighbor was indeed a tail connector)
        #                 hop.id = ''
        #                 hop.count = 1
        #                 self.tail_switch_timer = 2

        # If a connector was not found, update hop count if it has become a follower
        if robot_ids:
            for msg in self.other_team_msgs:

                # Robot is found to be a follower so delete entries from hops_dict with the robot's id
                if msg.id in robot_ids:

                    # Check if it is not a robot that it has just sent an accept message to
                    sent_accept = False
                    for send_msg in self.cmsg_to_resend:
                        if send_msg[1].to == msg.id:
                            sent_accept = True
                            robot_ids.remove(msg.id)
                            break

                    if not sent_accept:
                        # Find all keys that this robot appears in
                        team_keys = []
                        for id, hop in self.hops_dict.items():
                            if hop.id == msg.id:
                                team_keys.append(id)

                        # Delete the robot's ID and update hop count to 1
                        for key in team_keys:
                            self.hops_dict[key].id = ''
                            self.hops_dict[key].count = 1
                            print(f'deleting team in hops_dict: {key}')

                        # self.tail_switch_timer = 2 # TEMP hard-coded duration

                        robot_ids.remove(msg.id)

        if robot_ids:
            print(Fore.RED + f'robot_ids not empty for robot: {robot_ids}, ', end='')
            for id in robot_ids:
                print(Fore.RED + f'{id}, ', end='')
            print()

        # Update hop count
        for id, hop in self.hops_dict.items():
            previous_robot_id = hop.id

            if previous_robot_id:

                if previous_robot_id in robot_ids:
                    hop.count = 100000
                else:
                    # If it has just sent an accept, don't update count
                    team_to_check = id
                    if previous_robot_id in robot_messages:

                        previous_hop = robot_messages[previous_robot_id].hops[team_to_check]
                        sending_accept = False
                        for pair in self.cmsg_to_resend:
                            if pair[1].type == 'A' and pair[1].to == previous_robot_id:
                                sending_accept = True

                        if not sending_accept:
                            hop.count = previous_hop.count + 1 # Increment by 1

        # Extract updated connector IDs to check
        robot_ids.clear()
        for team, hop in self.hops_dict.items():
            if hop.id:
                robot_ids.add(hop.id)
        
        # Add hop information about teams that it hasn't seen before

        # Extract all teams it hasn't seen before i.e. not in hops_dict
        new_team_ids = set()
        combined_adjacent_msg = []
        for key, val in robot_messages.items():
            combined_adjacent_msg.append(val)

        combined_adjacent_msg.extend(self.other_team_msgs)
        for msg in combined_adjacent_msg:
            for id, hop in msg.hops.items():
                # Check if team_id exists in its hop_dict
                if not id in self.hops_dict:
                    # Add team_id if it doesn't exist
                    new_team_ids.add(id)

        # Add hop count to unseen team via adjacent connection

        # For each new ID, find the adjacent connector with the smallest hop count and register it to hops_dict
        for new_team_id in new_team_ids:
            min_hop_count = 100000
            min_hop_count_robot = ''
            for msg in combined_adjacent_msg:
                for id, hop in self.hops_dict.items():

                    if id == new_team_id:

                        if not min_hop_count_robot or hop.count < min_hop_count:
                            min_hop_count = hop.count
                            min_hop_count_robot = msg.id
                            print(Fore.YELLOW + f'[{self.id}] adding new entry to hops_dict for team {new_team_id} with hop {min_hop_count} by {min_hop_count_robot}')

            # Add new_team_id to hops_dict
            hop = HopMsg()
            hop.count = min_hop_count + 1
            hop.id = min_hop_count_robot
            self.hops_dict[new_team_id] = hop


    def check_network_modification(self):
        
        # Extract neighboring connector IDs to check
        robot_ids = set()
        for team, hop in self.hops_dict.items():
            if hop.id:
                robot_ids.add(hop.id)

        # Extract Messages from connectors that have the IDs found previously
        robot_messages = {}
        neighbor_messages = {}
        other_connector_messages = {}

        for msg in self.connector_msgs:
            # Find the next connector
            if msg.id in robot_ids:
                robot_messages[msg.id] = msg
                neighbor_messages[msg.id] = msg
                robot_ids.remove(msg.id)
            else:
                other_connector_messages[msg.id] = msg

        # Find the number of unique robots/teams this robot is connected with
        neighbor_ids = set()
        for id, msg in neighbor_messages.items():
            neighbor_ids.add(id)

        # Check if it should modify its connection to optimize the network
        if self.connector_switch_timer == 0 and self.network_change_timer == 0 and not self.notify_sent:
            
            change_connection = False
            
            # Check neighbor teams
            # Find the teamIDs of followers that are within its safety range
            nearby_msgs = self.other_team_msgs + self.other_leader_msgs
            for msg in nearby_msgs:
                dist = abs(msg.direction)
                if msg.team_id in self.nearby_teams:
                    if dist < self.nearby_teams[msg.team_id]:
                        self.nearby_teams[msg.team_id] = dist # Update dist if a shorter dist is found
                else:
                    self.nearby_teams[msg.team_id] = dist # Store team id and dist if it's under joining thres

            # For every nearbyTeam
                # For every neighbor connector
                    # if neighbor hop count to team = 1 && teamDist < neighborDist
                        # changeConnection = True
                        # networkChanges[team] = {old,new(team)}

            if not self.traveler_msgs: # Don't switch tail connectors while there are travelers nearby
                for team, dist in self.nearby_teams.items(): # Non-adjacent team
                    for neighbor_id, neighbor_msg in neighbor_messages.items():

                        # Count the number of connections the neighbor has
                        unique_connections = set()
                        for other_team, other_hop in neighbor_msg.hops.items():
                            if other_hop.count == 1:
                                unique_connections.add(str(other_team))
                            elif other_hop.id in unique_connections:
                                unique_connections.add(other_hop.id)

                        if len(unique_connections) > 2: # condNO3
                            try:
                                neighbor_dist_to_team = neighbor_msg.nearby_teams[team]
                            except KeyError as e:
                                print(f'KeyError - {e}')
                                neighbor_dist_to_team = 0

                            if neighbor_msg.hops[team].count == 1 and dist < neighbor_dist_to_team:
                                change_connection = True
                                team_place_holder = Message()
                                # team_place_holder.id = "T" + team
                                team_place_holder = team
                                self.network_changes[team] = ([neighbor_msg.id, team_place_holder])
                                break
                    
                    if change_connection:
                        break

            # Check neighbor connectors
            if not change_connection:
                for connector_id, connector_msg in other_connector_messages.items(): # Non-adjacent connector
                    for neighbor_id, neighbor_msg in neighbor_messages.items(): # Adjacent connector

                        connector_dist = abs(connector_msg.direction)
                        neighbor_dist = abs(neighbor_msg.direction)

                        if connector_dist < neighbor_dist: # condNO1
                            # Check whether the connector and neighbor are overlapping itself in the network

                            change_connector = False

                            for team, hop in self.hops_dict.items():

                                connector_exists = False
                                unique_connections = set()
                                for other_team, other_hop in neighbor_msg.hops.items():
                                    # Count the number of connections the neighbor has
                                    if other_hop.count == 1:
                                        unique_connections.add(str(other_team))
                                    elif not other_hop.id in unique_connections:
                                        unique_connections.add(other_hop.id)

                                    if other_hop.id == connector_msg.id: # condNO2
                                        connector_exists = True
                                        # break

                                # print(Fore.LIGHTYELLOW_EX + f'unique_connections: {unique_connections}, exist: {connector_exists}')

                                if len(unique_connections) > 2 and connector_exists: # condNO3
                                    change_connector = True
                                else:
                                    change_connector = False

                            if change_connector:
                                # Store potential network changes
                                for team, hop in self.hops_dict.items():
                                    if hop.id == neighbor_id:
                                        # To avoid connectors simultaneously changing the network, only allow connectors with larger IDs to make the change
                                        if self.id > connector_id:
                                            self.network_changes[team] = ([hop.id, connector_msg])


    def store_network_modification(self):
        # Check for network change messages and update accordingly
        for msg in self.connector_msgs:
            # Store messages with nmsg
            if msg.nmsg:
                self.nmsgs_received.append(msg)


    def update_hop_counts_follower(self):
        """Update hop count within team and to other teams"""

        # Get Connector messages that has connection to this team
        adjacent_connector_msgs = []
        for msg in self.connector_msgs:
            for id, hop in msg.hops.items():
                if id == self.team_id:
                    if hop.id == "" or hop.id in leader_id.values():
                        adjacent_connector_msgs.append(msg)
                        break

        # Get team IDs from team and adjacent connectors
        combined_msgs = adjacent_connector_msgs + [self.leader_msg] + self.team_msgs
        team_ids = set()
        for msg in combined_msgs:
            for id, hop in msg.hops.items():
                team_ids.add(id)
        if self.team_id in team_ids:
            team_ids.remove(self.team_id) # Delete its own team id

        # print(f'team ids: {team_ids}')

        for other_team_id in team_ids:

            # Loop connector (that has connections to this team)

            connector_found = False

            for msg in adjacent_connector_msgs:

                # If connector is a predecessor, use it to update its own hop count
                if other_team_id in msg.hops:
                    if msg.hops[other_team_id].id == '' or not msg.hops[other_team_id].id in leader_id.values():
                        
                        if not other_team_id in self.hops_dict:
                            self.hops_dict[other_team_id] = HopMsg()

                        self.hops_dict[other_team_id].count = msg.hops[other_team_id].count + 1
                        self.hops_dict[other_team_id].id = msg.id
                        self.hops_dict[other_team_id].resend_count = 0
                        connector_found = True
                        break

            if not connector_found:
                # Loop teammate and leader
                combined_team_msgs = self.team_msgs + [self.leader_msg]

                min_hop = HopMsg()
                min_resend_count = 100000
                for msg in combined_team_msgs:
                    if other_team_id in msg.hops and msg.hops[other_team_id].resend_count < min_resend_count:
                        min_hop = msg.hops[other_team_id]
                        min_resend_count = msg.hops[other_team_id].resend_count

                if min_resend_count < 100000:
                    self.hops_dict[other_team_id] = min_hop


    def flock(self):
        # print('[Robot {}] Action: flock'.format(self.id))

        repulse_msgs = []
        if not self.leader_msg.empty():
            repulse_msgs.append(self.leader_msg)
        repulse_msgs += self.team_msgs + self.other_leader_msgs + self.other_team_msgs + self.connector_msgs + self.traveler_msgs
        repulse_ids = {str(msg.id) for msg in repulse_msgs}

        team_force = self.get_team_flocking_vector()
        robot_force = self.get_robot_repulsion_vector(repulse_ids)
        obstacle_force = self.get_obstacle_repulsion_vector()
        sum_force = Worker.FOLLOWER_ATTRACTION * team_force + Worker.FOLLOWER_REPULSION * robot_force + Worker.FOLLOWER_OBSTACLE * obstacle_force

        # print("Sum vector: {}".format(sum_force))

        if abs(sum_force) > 0.1:
            return self.set_wheel_speeds(sum_force)
        elif abs(sum_force) == 0:
            return self.left, self.right
        else:
            return 0, 0


    def get_team_flocking_vector(self):
        res_vec = Vector2D(0,0)

        # print(self.neighbours)

        if not self.leader_msg.empty():
            # print("leader detected")
            robot_data = self.neighbours[str(self.leader_msg.id)]
            res_vec += get_vector(robot_data)

        else:
            if self.hop_count_to_leader ==  100000:
                return Vector2D(0,0)
            
            num_attract = 0

            for msg in self.team_msgs:
                # print(f"{msg.team_hop_count} - {self.hop_count_to_leader}")
                if msg.team_hop_count < self.hop_count_to_leader:
                    robot_data = self.neighbours[str(msg.id)]
                    res_vec += get_vector(robot_data)

                    num_attract += 1

            # print(f"num_attract: {num_attract}")
            if num_attract > 0:
                res_vec /= num_attract

        # TODO: Limit vector length to max speed

        return res_vec


    def get_robot_repulsion_vector(self, ids):

        # Repel other robots
        num_other_robots = 0
        res_vec = Vector2D(0,0)

        # print(Fore.RED + f'ids: {ids}')
        # print(Fore.RED + f'keys: {self.neighbours.keys()}')

        for key, value in self.neighbours.items():
            if key in ids:
                distance = value.range
                lj_force = self.generalized_lennard_jones_repulsion(distance)
                # print("LJ force: {}".format(lj_force))
                x = math.cos(math.radians(value.bearing)) * lj_force
                y = math.sin(math.radians(value.bearing)) * lj_force
                rep_vec = Vector2D(x, y)
                # print("Vector to repel: {}".format(rep_vec))
                res_vec += rep_vec

            num_other_robots += 1            

        if num_other_robots > 0:
            res_vec /= num_other_robots

        return res_vec


    def travel(self):

        # Add robots to repel
        repulse_msgs = []
        repulse_msgs = self.other_leader_msgs + self.other_team_msgs + self.connector_msgs + self.traveler_msgs
        repulse_ids = {str(msg.id) for msg in repulse_msgs}

        # Calculate overall force applied to the robot
        travel_force = self.get_chain_travel_vector()
        robot_force = self.get_robot_repulsion_vector(repulse_ids)
        obstacle_force = self.get_obstacle_repulsion_vector()

        sum_force = Worker.TRAVELER_ATTRACTION * travel_force + Worker.TRAVELER_REPULSION * robot_force + Worker.TRAVELER_OBSTACLE * obstacle_force
        # print(Fore.RED + f'sum_force: {sum_force}')

        if abs(sum_force) > 0.01:
            return self.set_wheel_speeds(sum_force)
        else:
            return 0, 0


    def get_chain_travel_vector(self):

        res_vec = Vector2D(0,0)
        
        # If a follower in the target team is visible, move towards them directly
        combined_msgs = self.other_team_msgs + self.other_leader_msgs

        num_team_members = 0
        for msg in combined_msgs:
            if msg.team_id == self.team_to_join:
                res_vec += msg.direction
                num_team_members += 1

        if num_team_members:
            res_vec /= num_team_members
            return res_vec

        # Sort connectors according to the hop count towards the target team (large -> small)
        sorted_connector_msgs = deepcopy(self.connector_msgs)
        # print(Fore.RED + f'{sorted_connector_msgs}')
        # print(Fore.RED + f'{self.team_to_join}')
        sorted_connector_msgs = sorted(sorted_connector_msgs, key=lambda msg: msg.hops[self.team_to_join].count)
        # print(Fore.RED + f'{sorted_connector_msgs}')

        # Find the next connector to move towards
        # for msg in sorted_connector_msgs:

        # Calculate target vector
        id = sorted_connector_msgs[0].id
        # print(Fore.RED + f'id: {id}')
        direction = get_vector(self.neighbours[str(id)])
        # print(Fore.RED + f'{direction}')
        margin = direction.rotate(-math.pi/2)
        # print(Fore.RED + f'{margin}')

        margin = margin.normalize() / 1000 # Convert to 1 mm
        margin *= 50 # TEMP: hard coded distance of 50mm to target connector
        # print(Fore.RED + f'{margin}. length: {abs(margin)}')

        res_vec = direction + margin
        # print(Fore.RED + f'{res_vec}')

        # TODO: Limit the length of the vector to the max speed?

        return res_vec


    def adjust_position(self):

        if self.current_state == RobotState.FOLLOWER:
            return 0, 0

        # Get robot messages that this robot is directly connected with
        neighbor_msgs = self.get_neighbors()

        # Add robots to repel from
        neighbor_ids = set()
        for key, msg in neighbor_msgs.items():
            neighbor_ids.add(msg.id)

        repulse_msgs = []
        for msg in self.connector_msgs:
            if msg.id in neighbor_ids:
                repulse_msgs.append(msg)
        # repulse_msgs += self.connector_msgs
        # repulse_ids = {str(msg.id) for msg in repulse_msgs}

        attract_neighbor_force = self.get_connector_attract_neighbor_vector(neighbor_msgs)
        attract_team_force = self.get_connector_attract_team_vector(neighbor_msgs)
        robot_force = self.get_robot_repulsion_vector(repulse_msgs)
        obstacle_force = self.get_obstacle_repulsion_vector()

        sum_force = Worker.CONNECTOR_ATTRACTION_TO_CONNECTOR*attract_neighbor_force + Worker.CONNECTOR_ATTRACTION_TO_TEAM*attract_team_force + Worker.CONNECTOR_REPULSION*robot_force + Worker.CONNECTOR_OBSTACLE*obstacle_force

        print(f'attract_neighbor_force: {attract_neighbor_force}')
        print(f'attract_team_force: {attract_team_force}')
        print(f'robot_force: {robot_force}')
        print(f'sum_force: {sum_force}')
        # print(f'length: {abs(sum_force)}')

        # # Calculate overall force applied to the robot
        # robot_force = self.get_connector_adjust_vector(neighbor_msgs)
        # # obstacle_force = self.get_obstacle_repulsion_vector()
        # sum_force = 1 * robot_force
        # print(Fore.RED + f'sum_force1: {sum_force}')

        # If it is a tail connector AND it loses connection with the team, stop
        team_lost = False
        for key, val in self.hops_dict.items():
            if val.count == 1:
                team_found = False
                for team, msg in neighbor_msgs.items():
                    if key == team:
                        team_found = True
                        break
                
                if not team_found:
                    team_lost = True
                    break

        if team_lost:
            sum_force = Vector2D(0,0)

        # print(Fore.RED + f'sum_force2: {sum_force}')

        # Set Wheel Speed
        if abs(sum_force) > 0.1:
            return self.set_wheel_speeds(sum_force)
        else:
            return 0, 0


    def get_neighbors(self):
        
        neighbor_msgs = {}

        # Get messages to check
        other_msgs = self.other_team_msgs + self.other_leader_msgs + self.connector_msgs

        for key, hop in self.hops_dict.items():

            team_to_check = key
            my_hop_count = hop.count
            robot_to_check = hop.id

            # shortest_dist = None
            neighbor_team_msgs = []

            for msg in other_msgs:

                if (msg.state == RobotState.LEADER or msg.state == RobotState.FOLLOWER) and my_hop_count == 1:

                    # For the team that it is a tail connector for
                    if msg.team_id == team_to_check:

                        dist = abs(msg.direction)

                        # Store the message with the shortest distance
                        if not team_to_check in neighbor_msgs or dist < abs(neighbor_msgs[team_to_check].direction):
                            # Store the distance to the team if this is the first member seen
                            neighbor_msgs[team_to_check] = msg
                        
                        neighbor_team_msgs.append(msg)

                elif msg.state == RobotState.CONNECTOR:

                    # For the team that it is NOT a tail connector for

                    # Is this connector my adjacent connector? If yes, record vector towards it
                    if msg.id == robot_to_check:
                        neighbor_msgs[team_to_check] = msg

            if my_hop_count == 1:
                # If it is a tail connector, use the average position of the team members
                if neighbor_team_msgs:
                    resVec = Vector2D(0,0)

                    # Get average position of the team
                    for msg in neighbor_team_msgs:
                        resVec += msg.direction
                    resVec /= len(neighbor_team_msgs)

                    # Replace the vector; use the average position of the team
                    neighbor_msgs[team_to_check].direction = resVec

        temp_dict = {key:msg.id for (key,msg) in neighbor_msgs.items()}
        print(Fore.BLUE + f'neighbor_msgs: {temp_dict}')

        return neighbor_msgs


    def get_connector_attract_neighbor_vector(self, neighbor_msgs):
        
        resVec = Vector2D(0,0)

        checked_neighbors = set()

        # Attraction to neighbors
        for team, msg in neighbor_msgs.items():
            if not msg.id in checked_neighbors:
                # If the distance to neighbors is critical, reduce the attraction towards team
                target = Worker.CONNECTOR_TARGET_DISTANCE
                modifier = 1

                if abs(msg.direction) > target:
                    modifier = (target+0.15 - target) / (target+0.15 - abs(msg.direction)) # TEMP: Hard-coded value. Assumes comm_range = 0.5 and target_dist = 0.35
                    
                # Add extra vector if it is too far
                robot = self.neighbours[str(msg.id)]
                x = math.cos(math.radians(robot['bearing'])) * robot['range']
                y = math.sin(math.radians(robot['bearing'])) * robot['range']
                resVec += Vector2D(x, y) * modifier

                checked_neighbors.add(msg.id)

        if neighbor_msgs:
            resVec /= len(neighbor_msgs)

        return resVec
    

    def get_connector_attract_team_vector(self, neighbor_msgs):
        
        # Attraction to team
        resVec = Vector2D(0,0)

        # Calculate the furthest neighbor distance
        furthest_neighbor_dist = 0
        for team, msg in neighbor_msgs.items():
            if msg.state == RobotState.CONNECTOR and abs(msg.direction) > furthest_neighbor_dist:
                furthest_neighbor_dist = abs(msg.direction)

        # If the distance to neighbors is critical, reduce the attraction towards team
        target = Worker.CONNECTOR_TARGET_DISTANCE
        modifier = 1
        if furthest_neighbor_dist > target:
            modifier = (target+0.075 - furthest_neighbor_dist) / (target+0.075 - target)

        # If smaller than zero, set to zero
        if modifier < 0:
            modifier = 0

        # Check whether it is connected to two neighbors
        neighbor_ids = set()
        for team, hop in self.hops_dict.items():
            neighbor_ids.add(hop.id)

        if len(neighbor_ids) < 3:
            num_team_attract = 0

            # Add attraction toward teams with hop count 1
            for team, hop in self.hops_dict.items():
                if hop.count == 1:
                    # Add the vector to the team
                    if team in neighbor_msgs:
                        resVec += neighbor_msgs[team].direction * modifier
                        num_team_attract += 1

            if num_team_attract:
                resVec /= num_team_attract

        return resVec


    ### Controllable events ###

    def _callback_moveFlock(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] MoveFlock")
        self.current_move_type = MoveType.FLOCK
        self.current_request = ConnectionMsg() # Clear existing request

    
    def _callback_moveChain(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] MoveChain")
        self.current_move_type = MoveType.TRAVEL
        self.current_request = ConnectionMsg() # Clear existing request

    
    def _callback_moveAdjust(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] MoveAdjust")
        self.current_move_type = MoveType.ADJUST

    
    # def _callback_taskStart(self, data):
    #     print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] TaskStart")
    #     self.performing_task = True

    
    # def _callback_taskStop(self, data):
    #     print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] TaskStop")
    #     self.performing_task = False

    
    def _callback_switchF(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] SwitchF")

        # Set new team_id
        if self.current_state == RobotState.CONNECTOR:
            for id, hop in self.hops_dict.items():
                if hop.count == 1:
                    self.team_id = id
                    break

            self.hops_dict.clear()

        elif self.current_state == RobotState.TRAVELER:
            self.team_id = self.team_to_join # Update its own team ID
            self.hops_dict.pop(self.team_id) # Delete entry of new team from hopsDict
        
        self.robot_to_switch = ''

        self.current_state = RobotState.FOLLOWER

        self.connector_switch_timer = 0
        self.network_change_timer = 0

    
    def _callback_switchC(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] SwitchC")

        # # Loop accept messages to see if it has received a message from a leader
        # accept_from_leader = False
        # for id, cmsg in self.current_accepts.items():
        #     if cmsg.origin == leader_id[self.team_id]:
        #         accept_from_leader = True
        #         break

        # if accept_from_leader: # Accept received from the leader

        #     # Add every other visible team to the hop map
        #     for msg in self.other_team_msgs:

        #         # Add hop count entry if not yet registered
        #         if not msg.team_id in self.hops_dict:
        #             hop = HopMsg()
        #             hop.count = 1
        #             self.hops_dict[msg.team_id] = hop

        #         self.hops_dict[msg.team_id].resend_count = 100000

        # else:   

        # # Accept received from a connector
        # team = 100000
        # for id, cmsg in self.current_accepts.items():
        #     team = id
        #     break

        # new_hops_dict = self.hops_copy[team]

        # Use the connector to generate its hop count to other teams
        self.hops_copy.pop(self.team_id, None) # Delete entry of its own team

        for id, hop in self.hops_copy.items(): # Loop to add hop count of 1 to each item
            hop.count += 1
            hop.id = self.current_accept.origin

        self.hops_dict = deepcopy(self.hops_copy) # Set to its hops

        # Set hop count to the team it is leaving to 1
        hop = HopMsg()
        hop.count = 1
        self.hops_dict[self.team_id] = hop

        # Reset variables
        self.current_accept = ConnectionMsg()
        self.hops_copy.clear()
        for id in self.team_shared_msg_dict:
            self.team_shared_msg_dict[id] = TeamSharedMsg()

        self.current_state = RobotState.CONNECTOR
        self.team_id = 100000

        self.connector_switch_timer = 4 # TODO: Hard-coded timesteps to wait until it can trigger a networkchange
        self.network_change_timer = 0


    def _callback_switchT(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] SwitchT")

        # Reset variables
        for id in self.team_shared_msg_dict:
            self.team_shared_msg_dict[id] = TeamSharedMsg()

        self.current_state = RobotState.TRAVELER
        self.team_id = 100000
        self.connector_switch_timer = 0
        self.network_change_timer = 0

    
    # def _callback_request(self, data):
    #     print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] Request")

    #     # Set requests to send
    #     for id, cond in self.role_switch_conditions_per_team.items():
    #         if cond.connector_conditions():
    #             cmsg = ConnectionMsg()
    #             cmsg.type = 'R'
    #             cmsg.origin = self.id
    #             cmsg.other_team = id
    #             if not self.team_shared_msg_dict[id].connector_id_downstream:
    #                 cmsg.to = leader_id[self.team_id]
    #                 cmsg.to_team = self.team_id
    #             else:
    #                 cmsg.to = self.team_shared_msg_dict[id].connector_id_downstream
    #                 # cmsg.to_team = None
                
    #             self.cmsg_to_resend.append([Worker.SEND_DURATION, cmsg]) # Transmit public event
    #             self.current_requests[id] = cmsg

    #     self.request_timer = Worker.WAIT_REQUEST_DURATION

    #     for key, val in self.current_requests.items():
    #         print(f'\t(to: {val.to}, for: {val.other_team})', end='')
    #     print()


    # def _callback_requestL(self, data):
    #     print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] RequestL")

    #     # Set requests to send
    #     for id, cond in self.role_switch_conditions_per_team.items():
    #         if cond.connector_conditions():
    #             cmsg = ConnectionMsg()
    #             cmsg.type = 'R'
    #             cmsg.origin = self.id
    #             cmsg.other_team = id
    #             cmsg.to = leader_id[self.team_id]
    #             cmsg.to_team = self.team_id
                
    #             self.cmsg_to_resend.append([Worker.SEND_DURATION, cmsg]) # Transmit public event
    #             self.current_requests[id] = cmsg

    #     self.request_timer = Worker.WAIT_REQUEST_DURATION

    #     for key, val in self.current_requests.items():
    #         print(f'\t(to: {val.to}, for: {val.other_team})', end='')
    #     print()


    def _callback_requestC(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] RequestC")

        # Set request to send
        for id, cond in self.role_switch_conditions_per_team.items():
            if cond.connector_conditions():
                cmsg = ConnectionMsg()
                cmsg.type = 'R'
                cmsg.origin = self.id
                cmsg.other_team = id
                cmsg.to = self.team_shared_msg_dict[id].connector_id_downstream
                if not cmsg.to:
                    cmsg.to = '12' # If the team has not identified the adjacent connector, send the message to 12, which is the default connector
                
                self.cmsg_to_resend.append([Worker.SEND_DURATION, cmsg]) # Transmit public event
                self.current_request = cmsg
                break # request for the first satisying team

        self.request_timer = Worker.WAIT_REQUEST_DURATION

        print(f'\t(to: {self.current_request.to}, for: {self.current_request.other_team})', end='')

    
    def _callback_respond(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] Respond")

        for id, msg in self.robots_to_accept.items():
            cmsg = ConnectionMsg()
            cmsg.type = 'A'
            cmsg.origin = self.id
            cmsg.to = msg.id
            cmsg.to_team = id
            self.cmsg_to_resend.append([Worker.SEND_RESPOND_DURATION, cmsg]) # Transmit public event

            # Update hop count to the team using the new connector
            self.hops_dict[id].count += 1
            self.hops_dict[id].id = msg.id

        for cmsg in self.cmsg_to_resend:
            print(f'\t(to: {cmsg[1].to}, in: {cmsg[1].to_team})', end='')
        print()

    
    def _callback_relay(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] Relay")

        for id, info in self.last_beat.items():
            if info[1] != 'N':
                self.rmsg_to_resend.append([Worker.SEND_DURATION, info[0]])


    def _callback_notifyNM(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] NotifyNM")

        # Set network change to broadcast
        for team, pair in self.network_changes.items():
            nmsg = NetworkChangeMsg()
            nmsg.team_id = team
            nmsg.prev_id = pair[0]
            nmsg.new_id = pair[1].id
            self.nmsg_to_resend.append([Worker.SEND_DURATION+2, nmsg]) # added extra timestep to send
            print(f'NOTIFY CHANGE FROM {nmsg.prev_id} TO {nmsg.new_id} FOR TEAM {team}')

        self.notify_sent = True


    def _callback_applyNM(self, data):
        print(Fore.LIGHTMAGENTA_EX + f"[Robot {self.id}] ApplyNM")

        if self.notify_sent:

            # Network change was found by this robot

            for team, hop in self.hops_dict.items():
                for team_nmsg, pair in self.network_changes.items():
                    if team == team_nmsg and hop.id == pair[0]:
                        # hop.id = pair[1].id
                        # hop.count = pair[1].hops[team].count + 1
                        ignore_duration = 4 # hard-coded duration

                        # if pair[1].id[0] == 'T':
                        if pair[1].id <= 10: # TEMP: Reserve numbers 10 or less as team ids
                            # Become a tail connector
                            hop.id = ""
                            hop.count = 1
                            print(f'UPDATED CONNECTION(SENT) FOR TEAM {team} TO (tail)')
                        else:
                            # Change connections between connectors
                            hop.id = pair[1].id
                            hop.count = pair[1].hops[team].count + 1
                            print(f'UPDATED CONNECTION(SENT) FOR TEAM {team} TO {hop.id}')

                        self.new_connections.append([ignore_duration, hop.id])

            self.notify_sent = False

        else:
            # Network change was received from another robot

            for msg in self.nmsgs_received:
                # Get set of teams the other robot is changing connections for
                team_ids = set()
                for nmsg in msg.nmsg:
                    team_ids.add(nmsg.team_id)
                
                for nmsg in msg.nmsg:

                    # Received connection removal; connect to new target
                    if nmsg.prev_id == self.id:
                        # if nmsg.new_id[0] != 'T':
                        if nmsg.new_id > 10: # TEMP: Reserve numbers 10 or less as team ids
                            for team, hop in self.hops_dict.items():
                                if hop.id == msg.id:
                                    hop.id = nmsg.new_id # Change team source to the new robot
                                    print(f'UPDATED CONNECTION(PREV) FOR TEAM {team} TO {hop.id}')
                        else:
                            team = int(nmsg.new_id[1:])
                            if self.hops_dict[team].id != msg.id:
                                self.hops_dict[team].id = msg.id
                                print(f'UPDATED CONNECTION(PREV) FOR TEAM {team} TO {msg.id}')

                    # Received connection addition; connect to sender
                    elif nmsg.new_id == self.id:
                        for team, hop in self.hops_dict.items():
                            if not team in team_ids:
                                if hop.id != msg.id:
                                    hop.id = msg.id # Change team source to sender for connections not modified by sender
                                    print(f'UPDATED CONNECTION(NEW) FOR TEAM {team} TO {hop.id}')

            self.nmsgs_received.clear() # empty the vector
        
        self.network_change_timer = 4 # TODO: Hard-coded timesteps to wait until it can trigger a networkchange

    
    ### Uncontrollable events ###

    def _check_condC(self, data):
        # If it satisfies the connector condition for any team, return True
        for id, cond in self.role_switch_conditions_per_team.items():
            if cond.connector_conditions():
                return True
        return False


    def _check_notCondC(self, data):
        # If it does not satisfy the connector condition for any team, return True
        for id, cond in self.role_switch_conditions_per_team.items():
            if cond.connector_conditions():
                return False
        return True


    # def _check_condC1(self, data):
    #     if self.role_switch_conditions_per_team:
    #         id = list(self.role_switch_conditions_per_team.keys())[0] # ASSUMING THERE IS ONLY ONE OTHER TEAM
    #         return self.role_switch_conditions_per_team[id].c1
    #     else:
    #         return False

    
    # def _check_notCondC1(self, data):
    #     if self.role_switch_conditions_per_team:
    #         id = list(self.role_switch_conditions_per_team.keys())[0] # ASSUMING THERE IS ONLY ONE OTHER TEAM
    #         return not self.role_switch_conditions_per_team[id].c1
    #     else:
    #         return True


    # def _check_condC2(self, data):
    #     if self.role_switch_conditions_per_team:
    #         id = list(self.role_switch_conditions_per_team.keys())[0] # ASSUMING THERE IS ONLY ONE OTHER TEAM
    #         return self.role_switch_conditions_per_team[id].c2
    #     else:
    #         return False

    
    # def _check_notCondC2(self, data):
    #     if self.role_switch_conditions_per_team:
    #         id = list(self.role_switch_conditions_per_team.keys())[0] # ASSUMING THERE IS ONLY ONE OTHER TEAM
    #         return not self.role_switch_conditions_per_team[id].c2
    #     else:
    #         return True

    # def _check_condC3(self, data):
    #     if self.role_switch_conditions_per_team:
    #         id = list(self.role_switch_conditions_per_team.keys())[0] # ASSUMING THERE IS ONLY ONE OTHER TEAM
    #         return self.role_switch_conditions_per_team[id].c3
    #     else:
    #         return False

    # def _check_notCondC3(self, data):
    #     if self.role_switch_conditions_per_team:
    #         id = list(self.role_switch_conditions_per_team.keys())[0] # ASSUMING THERE IS ONLY ONE OTHER TEAM
    #         return not self.role_switch_conditions_per_team[id].c3
    #     else:
    #         return True


    # def _check_nearC(self, data):
    #     # If connector exists nearby, return True
    #     return self.connector_msgs


    # def _check_notNearC(self, data):
    #     # If connector does not exist nearby, return False
    #     return not self.connector_msgs


    def _check_condF(self, data):
        # Get all conditions per team where f1 is true
        tail_teams = set()
        for key, cond in self.role_switch_conditions_per_team.items():
            if cond.f1:
                tail_teams.add(key)

        if tail_teams:
            for key in tail_teams:
                if not self.role_switch_conditions_per_team[key].f2:
                    # If it is needed by at least one adjacent connector, it cannot become a follower
                    return False
            
            # It was not needed by any connector or team, so it can be a follower
            return True

        return False


    def _check_notCondF(self, data):
        # Get all conditions per team where f1 is true
        tail_teams = set()
        for key, cond in self.role_switch_conditions_per_team.items():
            if cond.f1:
                tail_teams.add(key)

        if tail_teams:
            for key in tail_teams:
                if not self.role_switch_conditions_per_team[key].f2:
                    # If it is needed by at least one adjacent connector, it cannot become a follower
                    return True
            
            # It was not needed by any connector or team, so it can be a follower
            return False

        return False


    # def _check_condF1(self, data):
    #     # TEMP: evaluate both f1 and f2 together. Return true if both are true for at least one team
    #     for id, cond in self.role_switch_conditions_per_team.items():
    #         if cond.f1 and cond.f2:
    #             return True
    #     return False
    
    
    # def _check_notCondF1(self, data):
    #     # TEMP: evaluate both f1 and f2 together. Return false if both are true for at least one team
    #     for id, cond in self.role_switch_conditions_per_team.items():
    #         if cond.f1 and cond.f2:
    #             return False
    #     return True


    # def _check_condF2(self, data):
    #     # TEMP: evaluate both f1 and f2 together. Return true if both are true for at least one team
    #     for id, cond in self.role_switch_conditions_per_team.items():
    #         if cond.f1 and cond.f2:
    #             return True
    #     return False

    
    # def _check_notCondF2(self, data):
    #     # TEMP: evaluate both f1 and f2 together. Return false if both are true for at least one team
    #     for id, cond in self.role_switch_conditions_per_team.items():
    #         if cond.f1 and cond.f2:
    #             return False
    #     return True


    def _check__requestC(self, data):
        # print(f'check_request: {self.received_request}')
        return self.received_request


    def _check__respond(self, data):
        return self.received_accept or self.received_reject


    def _check_accept(self, data):
        return self.received_accept


    def _check_reject(self, data):
        return self.received_reject


    # def _check__start(self, data):
    #     if self.leader_signal == 1:
    #         return True
    #     return False


    # def _check__stop(self, data):
    #     if self.leader_signal == 0:
    #         return True
    #     return False


    def _check__message(self, data):
        for id, info in self.last_beat.items():
            if info[1] == 'L':
                print(Fore.YELLOW + 'Message received from leader')
                return True
        return False


    def _check__relay(self, data):
        for id, info in self.last_beat.items():
            if info[1] == 'W':
                print(Fore.YELLOW + 'Message received from worker')
                return True
        return False


    def _check__exchange(self, data):
        if self.robot_to_switch:
            return True
        return False


    def _check_chosen(self, data):
        if self.robot_to_switch == self.id:
            return True
        return False


    def _check_notChosen(self, data):
        if self.robot_to_switch != self.id:
            return True
        return False


    def _check_nearLF(self, data):
        return self.nearLF


    def _check_notNearLF(self, data):
        return not self.nearLF


    def _check_condNM(self, data):
        return bool(self.network_changes) 
        # return False

    
    def _check_notCondNM(self, data):
        return bool(self.network_changes) == False
        # return True


    def _check__notifyNM(self, data):
        
        for msg in self.nmsgs_received:

            # Get set of teams the other robot is changing connections for
            team_ids = set()
            for nmsg in msg.nmsg:
                team_ids.add(nmsg.team_id)

            for nmsg in msg.nmsg:

                # Received connection removal; connect to new target
                if nmsg.prev_id == self.id:
                    for team, hop in self.hops_dict.items():
                        # if nmsg.new_id[0] != 'T':
                        if nmsg.new_id > 10:
                            if hop.id == msg.id:
                                # There is a NetworkChangeMsg that alters the current connection
                                return True
                        else:
                            # Assuming a message to switch the tail connector role
                            try:
                                if team == int(nmsg.new_id[1:]) and not hop.id:
                                    # There is a NetworkChangeMsg that alters the current connection
                                    return True
                            except TypeError as e:
                                print(f'TypeError - {e}')

                # Received connection addition; connect to sender
                elif nmsg.new_id == self.id:
                    for team, hop in self.hops_dict.items():
                        if not team in team_ids:
                            if hop.id != msg.id:
                                # There is a NetworkChangeMsg that alters the current connection
                                return True
                            
        return False
    

    def _check_initC(self, data):
        return self.id == 12 and self.init_step_timer == 5
