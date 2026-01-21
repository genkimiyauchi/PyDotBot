import asyncio
from dataclasses import dataclass
import math
import os
from typing import Dict, List
import angles

from dotbot.examples.orca import (
    Agent,
    OrcaParams,
    compute_orca_velocity_for_agent,
)
from dotbot.examples.vec2 import Vec2, distance
from dotbot.models import (
    DotBotLH2Position,
    DotBotModel,
    DotBotMoveRawCommandModel,
    DotBotRgbLedCommandModel,
    DotBotWaypoints,
)
from dotbot.protocol import ApplicationType
from dotbot.rest import RestClient

from dotbot.examples.network_formation.controller import (
    Robot,
    Leader,
    Worker,
)

from vector2d import Vector2D

THRESHOLD = 30  # Acceptable distance error to consider a waypoint reached
DT = 0.05  # Control loop period (seconds)

# TODO: Measure these values for real dotbots
BOT_RADIUS = 0.03  # Physical radius of a DotBot (unit), used for collision avoidance
MAX_SPEED = 0.75  # Maximum allowed linear speed of a bot

(QUEUE_HEAD_X, QUEUE_HEAD_Y) = (
    0.1,
    0.1,
)  # World-frame (X, Y) position of the charging queue head
QUEUE_SPACING = (
    0.1  # Spacing between consecutive bots in the charging queue (along X axis)
)

SENSING_RANGE = 0.3  # Range within which neighbors can be sensed

@dataclass
class NeighborSensing:
    name: str
    range: float
    bearing: float
    position: Vec2


dotbot_controllers: Dict[str, Robot] = {}
dotbot_neighbors: Dict[str, Dict[str, NeighborSensing]] = {}

TARGETS = {
    1: Vec2(x=0.1, y=0.1),
    2: Vec2(x=0.9, y=0.1),
    3: Vec2(x=0.1, y=0.9),
    4: Vec2(x=0.9, y=0.9),
}

# class Controller:
#     def __init__(self, address: str, path: str):
#         # self.address = address

#         # # SCT initialization
#         # self.sct = SCT(path)
#         # self.add_callbacks()

#         self.waypoint_current = None

#         self.led = (0, 0, 0)  # initial LED color
#         # self.energy = 'high'  # initial energy level 


    # def set_work_waypoint(self, waypoint: DotBotLH2Position):
    #     self.waypoint_work = waypoint


    # def set_charge_waypoint(self, waypoint: DotBotLH2Position):
    #     self.waypoint_charge = waypoint


    # def set_current_position(self, position: DotBotLH2Position):
    #     self.position_current = position


    # def control_step(self):

    #     # # Calculate distance to work waypoint
    #     # dx = self.waypoint_work.x - self.position_current.x
    #     # dy = self.waypoint_work.y - self.position_current.y
    #     # self.dist_work = math.sqrt(dx * dx + dy * dy)

    #     # # Calculate distance to charge waypoint
    #     # dx = self.waypoint_charge.x - self.position_current.x
    #     # dy = self.waypoint_charge.y - self.position_current.y
    #     # self.dist_charge = math.sqrt(dx * dx + dy * dy)

    #     # Run SCT control step
    #     self.sct.run_step()


    # # Register callback functions to the generator player
    # def add_callbacks(self):

    #     # Automatic addition of callbacks
    #     # 1. Get list of events and list specifying whether an event is controllable or not.
    #     # 2. For each event, check controllable or not and add callback.

    #     events, controllability_list = self.sct.get_events()

    #     for event, index in events.items():
    #         is_controllable = controllability_list[index]
    #         stripped_name = event.split('EV_', 1)[1]    # Strip preceding string 'EV_'

    #         if is_controllable: # Add controllable event
    #             func_name = '_callback_{0}'.format(stripped_name)
    #             func = getattr(self, func_name)
    #             self.sct.add_callback(event, func, None, None)
    #         else:   # Add uncontrollable event
    #             func_name = '_check_{0}'.format(stripped_name)
    #             func = getattr(self, func_name)
    #             self.sct.add_callback(event, None, func, None)


    # Callback functions (controllable events)
    # def _callback_moveToWork(self, data: any):
    #     print(f'DotBot {self.address}. ACTION: moveToWork')
    #     self.waypoint_current = self.waypoint_work
    #     self.led = (0, 255, 0)  # Green LED when moving to work


    # def _callback_moveToCharge(self, data: any):
    #     print(f'DotBot {self.address}. ACTION: moveToCharge')
    #     self.waypoint_current = self.waypoint_charge
    #     self.led = (255, 0, 0)  # Red LED when moving to charge


    # def _callback_work(self, data: any):
    #     print(f'DotBot {self.address}. ACTION: work')
    #     self.energy = 'low'  # After working, energy level goes low


    # def _callback_charge(self, data: any):
    #     print(f'DotBot {self.address}. ACTION: charge')
    #     self.energy = 'high'  # After charging, energy level goes high


    # Callback functions (uncontrollable events)
    # def _check_atWork(self, data: any):
    #     if self.dist_work * 1000 < THRESHOLD:
    #         print(f'DotBot {self.address}. EVENT: atWork')
    #         return True
    #     return False


    # def _check_notAtWork(self, data: any):
    #     if self.dist_work * 1000 >= THRESHOLD:
    #         # print(f'DotBot {self.address}. EVENT: notAtWork')
    #         return True
    #     return False


    # def _check_atCharger(self, data: any):
    #     if self.dist_charge * 1000 < THRESHOLD:
    #         print(f'DotBot {self.address}. EVENT: atCharger')
    #         return True
    #     return False


    # def _check_notAtCharger(self, data: any):
    #     if self.dist_charge * 1000 >= THRESHOLD:
    #         # print(f'DotBot {self.address}. EVENT: notAtCharger')
    #         return True
    #     return False


    # def _check_lowEnergy(self, data: any):
    #     if self.energy == 'low':
    #         # print(f'DotBot {self.address}. EVENT: lowEnergy')
    #         return True
    #     return False


    # def _check_highEnergy(self, data: any):
    #     if self.energy == 'high':
    #         # print(f'DotBot {self.address}. EVENT: highEnergy')
    #         return True
    #     return False


def order_bots(
    dotbots: List[DotBotModel], base_x: int, base_y: int
) -> List[DotBotModel]:
    def key(bot: DotBotModel):
        dx = bot.lh2_position.x - base_x
        dy = bot.lh2_position.y - base_y
        return (dx * dx + dy * dy, bot.address)

    return sorted(dotbots, key=key)


def assign_goals(
    ordered: List[DotBotModel],
    head_x: int,
    head_y: int,
    spacing: int,
) -> Dict[str, dict]:
    goals = {}
    for i, bot in enumerate(ordered):
        
        # TODO: depending on the robot's current state (moving to base or work regions), assign a different goal
        
        goals[bot.address] = {
            "x": head_x,
            "y": head_y + i * spacing,
        }
    return goals


def preferred_vel(dotbot: DotBotModel, goal: Vec2 | None) -> Vec2:
    if goal is None:
        return Vec2(x=0, y=0)

    dx = goal["x"] - dotbot.lh2_position.x
    dy = goal["y"] - dotbot.lh2_position.y
    dist = math.sqrt(dx * dx + dy * dy)

    dist1000 = dist * 1000
    # If close to goal, stop
    if dist1000 < THRESHOLD:
        return Vec2(x=0, y=0)

    # Right-hand rule bias
    bias_angle = 0.0
    # Bot can only walk on a cone [-60, 60] in front of himself
    max_deviation = math.radians(60)

    # Convert bot direction into radians
    direction = direction_to_rad(dotbot.direction)

    # Angle to goal
    angle_to_goal = math.atan2(dy, dx) + bias_angle

    delta = angle_to_goal - direction
    # Wrap to [-π, +π]
    delta = math.atan2(math.sin(delta), math.cos(delta))

    # Clamp delta to [-MAX, +MAX]
    if delta > max_deviation:
        delta = max_deviation
    if delta < -max_deviation:
        delta = -max_deviation

    # Final allowed direction
    final_angle = direction + delta
    result = Vec2(
        x=math.cos(final_angle) * MAX_SPEED, y=math.sin(final_angle) * MAX_SPEED
    )
    return result


def direction_to_rad(direction: float) -> float:
    rad = (direction + 90) * math.pi / 180.0
    return math.atan2(math.sin(rad), math.cos(rad))  # normalize to [-π, π]


async def compute_orca_velocity(
    agent: Agent,
    neighbors: List[Agent],
    params: OrcaParams,
) -> Vec2:
    return compute_orca_velocity_for_agent(agent, neighbors, params)


# Exchange messages between robots that are within their communication ranges
def local_communication():
    for id, controller in dotbot_controllers.items():
        if id in dotbot_neighbors:
            for other_id in dotbot_neighbors[id]:
                dotbot_controllers[id].messages[other_id] = dotbot_controllers[other_id].msg

            print(f'Messages for DotBot {id}: {dotbot_controllers[id].messages[other_id].id}')


async def main() -> None:
    params = OrcaParams(time_horizon=DT)
    url = os.getenv("DOTBOT_CONTROLLER_URL", "localhost")
    port = os.getenv("DOTBOT_CONTROLLER_PORT", "8000")
    use_https = os.getenv("DOTBOT_CONTROLLER_USE_HTTPS", False)
    client = RestClient(url, port, use_https)

    # sct_path = os.getenv("DOTBOT_SCT_PATH", "dotbot/examples/network_formation/models/supervisor.yaml")

    dotbots = await client.fetch_active_dotbots()

    # Initialization
    for i, dotbot in enumerate(dotbots):

        # # Init controller
        # controller = Controller(dotbot.address, sct_path)
        # dotbot_controllers[dotbot.address] = controller     
        # 

        # TODO:
        # - Load experiment params (xml) not needed? Add config at the top of this file?
        # [ ] Init Leader
        #    [x] Init SCT
        #    [x] Gather messages
        #    [x] Provide messages
        #    [ ] Motion to target
        # [ ] Init Followers   

        # Init controllers
        dotbot_controllers[dotbot.address] = Leader(dotbot.address, team_id=i+1)

        print(f'team_id for DotBot {dotbot.address}: {dotbot_controllers[dotbot.address].team_id}')

        target = TARGETS[dotbot_controllers[dotbot.address].team_id]
        dotbot_controllers[dotbot.address].waypoints.put(Vector2D(target.x, target.y))

        # Cosmetic: all bots are red
        await client.send_rgb_led_command(
            address=dotbot.address,
            command=DotBotRgbLedCommandModel(red=255, green=0, blue=0),
        )

    # # Set work and charge goals for each robot
    # sorted_bots = order_bots(dotbots, QUEUE_HEAD_X, QUEUE_HEAD_Y)
    # base_goals = assign_goals(sorted_bots, QUEUE_HEAD_X, QUEUE_HEAD_Y, QUEUE_SPACING)
    # work_goals = assign_goals(sorted_bots, QUEUE_HEAD_X+0.8, QUEUE_HEAD_Y, QUEUE_SPACING)

    # for address, controller in dotbot_controllers.items():
    #     goal = base_goals[address]
    #     waypoint_charge = DotBotLH2Position(x=goal['x'], y=goal['y'], z=0)
    #     controller.set_charge_waypoint(waypoint_charge)

    #     goal = work_goals[address]
    #     waypoint_work = DotBotLH2Position(x=goal['x'], y=goal['y'], z=0)
    #     controller.set_work_waypoint(waypoint_work)

    # Simulation loop
    while True:

        # print('\n', end='')

        # Get position of all robots
        dotbots = await client.fetch_active_dotbots()

        dotbot_neighbors.clear()

        # Process neighbor messages
        for dotbot in dotbots:

            for other_dotbot in dotbots:

                if dotbot.address != other_dotbot.address: # Don't check against itself

                    range = distance(dotbot.lh2_position, other_dotbot.lh2_position)

                    if range < SENSING_RANGE:

                        dotbot_neighbors.setdefault(dotbot.address, {})[other_dotbot.address] = NeighborSensing(
                            name=other_dotbot.address,
                            range=range,
                            bearing=angles.normalize(math.degrees(math.atan2(
                                other_dotbot.lh2_position.y - dotbot.lh2_position.y,
                                other_dotbot.lh2_position.x - dotbot.lh2_position.x,
                            ))),
                            position=Vector2D(x=other_dotbot.lh2_position.x, y=other_dotbot.lh2_position.y)
                        )

            dotbot_controllers[dotbot.address].neighbours = dotbot_neighbors.get(dotbot.address, {})

            print(f'Neighbors for DotBot {dotbot.address}: {dotbot_controllers[dotbot.address].neighbours}')

        print(f'dotbot_neighbors: {dotbot_neighbors}')
        
        # Exchange messages between robots within communication range
        local_communication()

        for id, controller in dotbot_controllers.items():
            controller.control_step()

        goals: Dict[str, Dict[str, float]] = dict()
        agents: Dict[str, Agent] = {}

        for bot in dotbots:

            print(f'current_goal for DotBot {bot.address}: {dotbot_controllers.get(bot.address).current_goal}')

            # Get current goals
            if dotbot_controllers.get(bot.address).current_goal is not None:
                goals[bot.address] = {
                    "x": dotbot_controllers[bot.address].current_goal.x,
                    "y": dotbot_controllers[bot.address].current_goal.y,
                }
                
            agents[bot.address] = Agent(
                id=bot.address,
                    position=Vec2(x=bot.lh2_position.x, y=bot.lh2_position.y),
                    velocity=Vec2(x=0, y=0),
                    radius=BOT_RADIUS,
                    direction=bot.direction,
                    max_speed=MAX_SPEED,
                    preferred_velocity=preferred_vel(
                        dotbot=bot, goal=goals.get(bot.address)
                    ),
                )

        # Run controller for each robot
        for dotbot in dotbots:
            agent = agents[dotbot.address]
            pos = dotbot.lh2_position
            print(f"DotBot {dotbot.address}: Position ({pos.x:.2f}, {pos.y:.2f}), Direction {dotbot.direction:.2f}°")

            # Run controller
            controller = dotbot_controllers[dotbot.address]
            controller.position = Vector2D(pos.x, pos.y) # update position
            controller.control_step() # run SCT step

            # Get current goal
            goal = controller.current_goal
            if goal is not None:
                goals[dotbot.address] = {
                    "x": goal.x,
                    "y": goal.y,
                }

            # Send goal
            neighbors = [neighbor for neighbor in agents.values() if neighbor.id != agent.id]

            orca_vel = await compute_orca_velocity(
                agent, neighbors=neighbors, params=params
            )
            STEP_SCALE = 0.1
            step = Vec2(x=orca_vel.x * STEP_SCALE, y=orca_vel.y * STEP_SCALE)

            # ---- CLAMP STEP TO GOAL DISTANCE ----
            goal = goals.get(agent.id)
            if goal is not None:
                dx = goal["x"] - agent.position.x
                dy = goal["y"] - agent.position.y
                dist_to_goal = math.hypot(dx, dy)

                step_len = math.hypot(step.x, step.y)
                if step_len > dist_to_goal and step_len > 0:
                    scale = dist_to_goal / step_len
                    step = Vec2(x=step.x * scale, y=step.y * scale)
            # ------------------------------------

            waypoints = DotBotWaypoints(
                threshold=THRESHOLD,
                waypoints=[
                    DotBotLH2Position(
                        x=agent.position.x + step.x, y=agent.position.y + step.y, z=0
                    )
                ],
            )
            await client.send_waypoint_command(
                address=agent.id,
                application=ApplicationType.DotBot,
                command=waypoints,
            )
            led = controller.led_colour
            await client.send_rgb_led_command(
                address=dotbot.address,
                command=DotBotRgbLedCommandModel(red=led[0], green=led[1], blue=led[2]),
            )

    return None


if __name__ == "__main__":
    asyncio.run(main())
