import asyncio
import math
import os
from time import time
from typing import Dict, List

from dotbot.examples.vec2 import Vec2
from dotbot.models import (
    DotBotLH2Position,
    DotBotModel,
    DotBotMoveRawCommandModel,
    DotBotRgbLedCommandModel,
    DotBotWaypoints,
    WSRgbLed,
    WSMoveRaw,
    WSWaypoints,
)
from dotbot.protocol import ApplicationType
from dotbot.rest import RestClient, rest_client
from dotbot.websocket import DotBotWsClient

from dotbot.examples.minimum_naming_game.controller import Controller

import numpy as np
import random
from scipy.spatial import cKDTree

COMM_RANGE=0.11
THRESHOLD=1

dotbot_controllers = dict()


async def main() -> None:
    url = os.getenv("DOTBOT_CONTROLLER_URL", "localhost")
    port = os.getenv("DOTBOT_CONTROLLER_PORT", "8000")
    use_https = os.getenv("DOTBOT_CONTROLLER_USE_HTTPS", False)
    sct_path = os.getenv("DOTBOT_SCT_PATH", "dotbot/examples/minimum_naming_game/models/supervisor.yaml")

    async with rest_client(url, port, use_https) as client:
        dotbots = await client.fetch_active_dotbots()

        # print(len(dotbots), "dotbots connected.")

        # Initialization
        for dotbot in dotbots:

            # Init controller
            controller = Controller(dotbot.address, sct_path)
            dotbot_controllers[dotbot.address] = controller    
            # print(f'type of controller: {type(controller)} for DotBot {dotbot.address}')   

        # 1. Extract positions into a list of [x, y] coordinates
        # This loop iterates through your dotbot list and grabs the lh2_position attributes
        coords = [[dotbot.lh2_position.x, dotbot.lh2_position.y] for dotbot in dotbots]

        # 2. Convert the list to a NumPy array
        # The structure will be (N, 2), where N is the number of dotbots
        positions = np.array(coords)

        # 3. Build the KD-Tree
        # This tree can now be used for fast spatial queries (like finding neighbors)
        tree = cKDTree(positions)

        ws = DotBotWsClient(url, port)
        await ws.connect()
        try:

            counter = 0

            while True:
                print("Step", counter)

                # ------ ONLY FOR MOVEMENT ------
                dotbots = await client.fetch_active_dotbots()

                # 1. Extract positions into a list of [x, y] coordinates
                # This loop iterates through your dotbot list and grabs the lh2_position attributes
                coords = [[dotbot.lh2_position.x, dotbot.lh2_position.y] for dotbot in dotbots]

                # 2. Convert the list to a NumPy array
                # The structure will be (N, 2), where N is the number of dotbots
                positions = np.array(coords)

                # 3. Build the KD-Tree
                # This tree can now be used for fast spatial queries (like finding neighbors)
                tree = cKDTree(positions)
                # -----------------------------

                for dotbot in dotbots:

                    controller = dotbot_controllers[dotbot.address]
                    controller.position = dotbot.lh2_position
                    controller.direction = dotbot.direction

                    # print(f'Controller position: {controller.position}, direction: {controller.direction}')

                    # 1. Query the tree for indices of neighbors
                    # This returns a list of integers representing the index in your 'dotbots' list
                    neighbor_indices = tree.query_ball_point([dotbot.lh2_position.x, dotbot.lh2_position.y], r=COMM_RANGE)
                    
                    # 2. Convert indices back into actual DotBot objects
                    # (We filter out the speaker themselves so they don't "hear" their own broadcast)
                    neighbors = [
                        dotbots[idx] for idx in neighbor_indices 
                        if dotbots[idx].address != dotbot.address
                    ]

                    # DEBUGGING: print neighbors
                    # print(f'neighbour of {dotbot.address}: {[n.address for n in neighbors]}')

                    # 3. If there are neighbors broadcasting, pick ONE randomly to listen to
                    if neighbors:
                        selected_neighbor = dotbot_controllers[random.choice(neighbors).address]

                        # Share the word: take the neighbor's chosen word index
                        if selected_neighbor.w_index != 0:
                            controller.received_word = selected_neighbor.w_index
                        
                            # Set the flags so the robot knows it has a new message to process
                            controller.new_word_received = True
                            controller.received_word_checked = False
                        
                    # Update controller's neighbor list
                    controller.neighbors = neighbors
                        
                    # Run controller
                    controller.control_step() # run SCT step

                    waypoints = DotBotWaypoints(
                        threshold=THRESHOLD,
                        waypoints=[
                            DotBotLH2Position(
                                x=dotbot.lh2_position.x + round(controller.vector[0], 2), y=dotbot.lh2_position.y + round(controller.vector[1], 2), z=0
                            )
                        ],
                    )
                    await client.send_waypoint_command(
                        address=dotbot.address,
                        application=ApplicationType.DotBot,
                        command=waypoints,
                    )

                    await ws.send(
                        WSRgbLed(
                            cmd="rgb_led",
                            address=dotbot.address,
                            application=ApplicationType.DotBot,
                            data=DotBotRgbLedCommandModel(
                                red=controller.led[0],
                                green=controller.led[1],
                                blue=controller.led[2],
                            ),
                        )
                    )


                # await asyncio.sleep(0.1)
                counter += 1
        finally:
            await ws.close()

    return None


if __name__ == "__main__":
    asyncio.run(main())
