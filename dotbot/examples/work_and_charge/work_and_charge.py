import asyncio
import math
import os
import time
from typing import Dict, List

from dotbot.examples.orca import (
    Agent,
    OrcaParams,
    compute_orca_velocity_for_agent,
)
from dotbot.examples.vec2 import Vec2
from dotbot.models import (
    DotBotLH2Position,
    DotBotModel,
    DotBotMoveRawCommandModel,
    DotBotRgbLedCommandModel,
    DotBotWaypoints,
    WSRgbLed,
    WSWaypoints,
)
from dotbot.protocol import ApplicationType
from dotbot.rest import RestClient, rest_client
from dotbot.websocket import DotBotWsClient

from dotbot.examples.sct import SCT
from dotbot.examples.work_and_charge.controller import Controller

import numpy as np
from scipy.spatial import cKDTree

ORCA_RANGE = 30

THRESHOLD = 30  # Acceptable distance error to consider a waypoint reached
DT = 0.05  # Control loop period (seconds)

# TODO: Measure these values for real dotbots
BOT_RADIUS = 0.03  # Physical radius of a DotBot (unit), used for collision avoidance
MAX_SPEED = 0.075  # Maximum allowed linear speed of a bot

(QUEUE_HEAD_X, QUEUE_HEAD_Y) = (
    0.1,
    0.1,
)  # World-frame (X, Y) position of the charging queue head
QUEUE_SPACING = (
    0.06  # Spacing between consecutive bots in the charging queue (along X axis)
)

dotbot_controllers = dict()


class BlockProfiler:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.starts = {}
        self.averages = {}  # Moving averages for blocks
        self.currents = {}   # Instantaneous values for blocks
        self.avg_total_ms = 0.0
        self.iteration_count = 0

    def start(self, name):
        self.starts[name] = time.perf_counter()

    def stop(self, name):
        if name not in self.starts:
            return 0
        
        duration = (time.perf_counter() - self.starts[name]) * 1000
        self.currents[name] = duration
        
        # Update EMA
        if name not in self.averages:
            self.averages[name] = duration
        else:
            self.averages[name] = (self.alpha * duration) + (1 - self.alpha) * self.averages[name]
        return duration

    def update_total(self, total_ms):
        self.iteration_count += 1
        if self.iteration_count == 1:
            self.avg_total_ms = total_ms
        else:
            self.avg_total_ms = (self.alpha * total_ms) + (1 - self.alpha) * self.avg_total_ms
        return self.avg_total_ms


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


async def main() -> None:
    params = OrcaParams(time_horizon=5 * DT, time_step=DT)
    url = os.getenv("DOTBOT_CONTROLLER_URL", "localhost")
    port = os.getenv("DOTBOT_CONTROLLER_PORT", "8000")
    use_https = os.getenv("DOTBOT_CONTROLLER_USE_HTTPS", False)
    sct_path = os.getenv("DOTBOT_SCT_PATH", "dotbot/examples/work_and_charge/models/supervisor.yaml")
    async with rest_client(url, port, use_https) as client:
        dotbots = await client.fetch_active_dotbots()

        # Initialization
        for dotbot in dotbots:

            # Init controller
            controller = Controller(dotbot.address, sct_path)
            dotbot_controllers[dotbot.address] = controller        

            # Cosmetic: all bots are red
            await client.send_rgb_led_command(
                address=dotbot.address,
                command=DotBotRgbLedCommandModel(red=255, green=0, blue=0),
            )

        # Set work and charge goals for each robot
        # sorted_bots = order_bots(dotbots, QUEUE_HEAD_X, QUEUE_HEAD_Y)
        sorted_bots = sorted(dotbots, key=lambda bot: bot.address)
        base_goals = assign_goals(sorted_bots, QUEUE_HEAD_X, QUEUE_HEAD_Y, QUEUE_SPACING)
        work_goals = assign_goals(sorted_bots, QUEUE_HEAD_X+0.8, QUEUE_HEAD_Y, QUEUE_SPACING)

        for address, controller in dotbot_controllers.items():
            goal = base_goals[address]
            waypoint_charge = DotBotLH2Position(x=goal['x'], y=goal['y'], z=0)
            controller.set_charge_waypoint(waypoint_charge)

            goal = work_goals[address]
            waypoint_work = DotBotLH2Position(x=goal['x'], y=goal['y'], z=0)
            controller.set_work_waypoint(waypoint_work)

        avg_loop_time = 0.0
        iteration_count = 0
        alpha = 0.1  # Smoothing factor (0.1 means 10% new data, 90% history)

        profiler = BlockProfiler(alpha=0.1)

        while True:
            try:
                ws = DotBotWsClient(url, port)
                await ws.connect()

                while True:
                    iter_start = time.perf_counter()

                    profiler.start('update-dotbots')
                    dotbots = await client.fetch_active_dotbots()
                    time_update_dotbots = profiler.stop('update-dotbots')

                    profiler.start('run-dotbots')
                    goals = dict()
                    agents: Dict[str, Agent] = {}

                    for bot in dotbots:

                        # print(f'waypoint_current for DotBot {bot.address}: {dotbot_controllers.get(bot.address).waypoint_current}')

                        # Get current goals
                        if dotbot_controllers.get(bot.address).waypoint_current is not None:
                            goals[bot.address] = {
                                "x": dotbot_controllers[bot.address].waypoint_current.x,
                                "y": dotbot_controllers[bot.address].waypoint_current.y,
                            }
                            
                        agents[bot.address] = Agent(
                            id=bot.address,
                                position=Vec2(x=bot.lh2_position.x, y=bot.lh2_position.y),
                                velocity=Vec2(x=0, y=0),
                                radius=BOT_RADIUS,
                                max_speed=MAX_SPEED,
                                preferred_velocity=preferred_vel(
                                    dotbot=bot, goal=goals.get(bot.address)
                                ),
                            )
                        
                    # Prepare coordinates for all agents
                    # Extract [x, y] for every agent in the same order
                    agent_list = list(agents.values())
                    positions = np.array([[a.position.x, a.position.y] for a in agent_list])

                    # Build the KD-Tree
                    tree = cKDTree(positions)

                    # Run controller for each robot
                    for dotbot in dotbots:
                        agent = agents[dotbot.address]
                        pos = dotbot.lh2_position
                        # print(f"DotBot {dotbot.address}: Position ({pos.x:.2f}, {pos.y:.2f}), Direction {dotbot.direction:.2f}°")

                        # Run controller
                        controller = dotbot_controllers[dotbot.address]
                        controller.set_current_position(pos) # update position
                        controller.control_step() # run SCT step

                        # Get current goal
                        goal = controller.waypoint_current
                        goals[dotbot.address] = {
                            "x": goal.x,
                            "y": goal.y,
                        }

                        ### Send goal

                        # Without KD-Tree
                        # local_neighbors = [neighbor for neighbor in agents.values() if neighbor.id != agent.id]

                        # Using KD-Tree: Prepare neighbor list using KD-Tree
                        neighbor_indices = tree.query_ball_point([agent.position.x, agent.position.y], r=ORCA_RANGE)
                        local_neighbors = [agent_list[idx] for idx in neighbor_indices if agent_list[idx].id != agent.id]

                        if not local_neighbors:
                            orca_vel = agent.preferred_velocity
                        else:
                            orca_vel = await compute_orca_velocity(
                                agent, neighbors=local_neighbors, params=params
                            )
                        step = Vec2(x=orca_vel.x, y=orca_vel.y)

                        # print(f'goal for DotBot {dotbot.address}: ({goal.x:.2f}, {goal.y:.2f}), step: ({step.x:.4f}, {step.y:.4f})')

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
                        await ws.send(
                            WSWaypoints(
                                cmd="waypoints",
                                address=agent.id,
                                application=ApplicationType.DotBot,
                                data=waypoints,
                            )
                        )
                        await ws.send(
                            WSRgbLed(
                                cmd="rgb_led",
                                address=agent.id,
                                application=ApplicationType.DotBot,
                                data=DotBotRgbLedCommandModel(
                                    red=controller.led[0],
                                    green=controller.led[1],
                                    blue=controller.led[2],
                                ),
                            )
                        )
                    time_run_dotbots = profiler.stop('run-dotbots')

                    # Stop Total Timer
                    iter_end = time.perf_counter()
                    total_ms = (iter_end - iter_start) * 1000
                    iteration_count += 1

                    # Calculate EMA for Total Time
                    if iteration_count == 1:
                        avg_total_ms = total_ms
                    else:
                        avg_total_ms = (alpha * total_ms) + (1 - alpha) * avg_total_ms

                    # Calculate Ratios
                    total_ms = (time.perf_counter() - iter_start) * 1000
                    avg_total = profiler.update_total(total_ms)

                    # Output
                    print(f"\rIter: {profiler.iteration_count:04} | "
                            f"TOTAL: {total_ms:6.2f} (Avg: {avg_total:6.2f}) | "
                            f"UPDATE DOTBOTS: {profiler.currents['update-dotbots']:6.2f} (Avg: {profiler.averages['update-dotbots']:6.2f}) | "
                            f"RUN DOTBOTS: {profiler.currents['run-dotbots']:6.2f} (Avg: {profiler.averages['run-dotbots']:6.2f})", end="")

            except Exception as e:
                print(f"Connection lost: {e}")
                print("Retrying in 3 seconds...")
                await asyncio.sleep(3)  # Wait before trying to reconnect
            finally:
                await ws.close()

    return None


if __name__ == "__main__":
    asyncio.run(main())
