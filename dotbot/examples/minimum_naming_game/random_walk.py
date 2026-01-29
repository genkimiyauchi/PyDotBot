import math

from dotbot.models import DotBotModel


class WheelTurningParams:
    # State Constants
    NO_TURN = 0
    SOFT_TURN = 1
    HARD_TURN = 2

    def __init__(self):
        self.MaxSpeed = 100
        self.TurningMechanism = self.NO_TURN
        # Thresholds in Radians (Approx: 10°, 30°, 120°)
        self.NoTurnAngleThreshold = math.radians(10)
        self.SoftTurnOnAngleThreshold = math.radians(30)
        self.HardTurnOnAngleThreshold = math.radians(120)


def set_wheel_speeds_from_vector(vx, vy, current_direction_deg, wheel_turning_params):
    # 1. Calculate Target Angle in your system (0 = -X, 90 = +Y)
    # atan2(y, -x) makes -X the zero-base
    target_rad = math.atan2(vy, -vx)
    
    # 2. Convert current robot direction to Radians
    current_rad = math.radians(current_direction_deg)
    
    # 3. Calculate signed difference (Smallest angle to turn)
    # This result is between -PI and +PI
    angle_diff = (target_rad - current_rad + math.pi) % (2 * math.pi) - math.pi
    
    # --- The rest of your C++ logic follows ---
    abs_angle = abs(angle_diff)
    heading_length = math.sqrt(vx**2 + vy**2)
    base_speed = min(heading_length, wheel_turning_params.MaxSpeed)

    # State machine logic (Hysteresis)
    if wheel_turning_params.TurningMechanism == wheel_turning_params.HARD_TURN:
        if abs_angle <= wheel_turning_params.SoftTurnOnAngleThreshold:
            wheel_turning_params.TurningMechanism = wheel_turning_params.SOFT_TURN
    elif wheel_turning_params.TurningMechanism == wheel_turning_params.SOFT_TURN:
        if abs_angle > wheel_turning_params.HardTurnOnAngleThreshold:
            wheel_turning_params.TurningMechanism = wheel_turning_params.HARD_TURN
        elif abs_angle <= wheel_turning_params.NoTurnAngleThreshold:
            wheel_turning_params.TurningMechanism = wheel_turning_params.NO_TURN
    elif wheel_turning_params.TurningMechanism == wheel_turning_params.NO_TURN:
        if abs_angle > wheel_turning_params.NoTurnAngleThreshold:
            wheel_turning_params.TurningMechanism = wheel_turning_params.SOFT_TURN
        if abs_angle > wheel_turning_params.HardTurnOnAngleThreshold:
            wheel_turning_params.TurningMechanism = wheel_turning_params.HARD_TURN

    # Speed Calculation
    if wheel_turning_params.TurningMechanism == wheel_turning_params.NO_TURN:
        s1, s2 = base_speed, base_speed
    elif wheel_turning_params.TurningMechanism == wheel_turning_params.SOFT_TURN:
        # One wheel at base_speed, the other slowed down based on angle
        factor = (wheel_turning_params.HardTurnOnAngleThreshold - abs_angle) / wheel_turning_params.HardTurnOnAngleThreshold
        s1 = base_speed * factor
        s2 = base_speed  # Keep one wheel at full intended speed
    else: # HARD_TURN
        s1 = -wheel_turning_params.MaxSpeed
        s2 = wheel_turning_params.MaxSpeed

    # Assign Left/Right
    if angle_diff > 0: # Target is to the Left
        left, right = s1, s2
    else: # Target is to the Right
        left, right = s2, s1

    return int(round(left * 100)), int(round(right * 100))


def random_walk(position_x: float, position_y: float, direction: float, neighbors: list[DotBotModel]) -> list[float]:
    """
    Python implementation of Random Walk with Boundary Avoidance.
    Arena limits: x, y in [0.0, 1.0]
    """
    UNIT_SPEED = 0.1
    MARGIN = 0.1          # Trigger turn when within 10% of any edge
    
    # 1. Identify if any neighbor is too close
    neighbor_collision = False
    if neighbors:
        neighbor_collision = True

    # 2. Identify if any arena boundary is violated
    curr_x = position_x
    curr_y = position_y
    
    wall_collision = (curr_x < MARGIN or curr_x > (1.0 - MARGIN) or 
                      curr_y < MARGIN or curr_y > (1.0 - MARGIN))

    # 3. Determine "Local" movement
    # local_v[0] is longitudinal (forward), local_v[1] is lateral (sideways)
    local_v = [0.0, 0.0]

    if neighbor_collision or wall_collision:

        if wall_collision:
            # Decide direction of repulsion (Left or Right)
            # For simplicity, we can use the specific wall/neighbor location
            if (curr_x < MARGIN):
                local_v[0] += UNIT_SPEED
            if (curr_x > (1.0 - MARGIN)):
                local_v[0] += -UNIT_SPEED
            if (curr_y < MARGIN):
                local_v[1] += UNIT_SPEED
            if (curr_y > (1.0 - MARGIN)):
                local_v[1] += -UNIT_SPEED
            
        if neighbor_collision:
            avg_dx = sum(n.lh2_position.x - curr_x for n in neighbors)
            avg_dy = sum(n.lh2_position.y - curr_y for n in neighbors)

            mag = math.sqrt(avg_dx**2 + avg_dy**2)
            if mag > 0:
                # Add "Away" vector to our existing global movement
                local_v[0] -= (avg_dx / mag) * UNIT_SPEED
                local_v[1] -= (avg_dy / mag) * UNIT_SPEED
                # print(f"Neighbor avoidance vector: {local_v} from neighbors {[n.address for n in neighbors]}")

        # Final Step: Normalize so we don't go double speed in corners
        total_mag = math.sqrt(local_v[0]**2 + local_v[1]**2)
        if total_mag > 0:
            return (
                (local_v[0] / total_mag) * UNIT_SPEED,
                (local_v[1] / total_mag) * UNIT_SPEED,
            )
        return (0.0, 0.0)
            
    else:
        local_v = [UNIT_SPEED, 0.0] # Normal forward motion

        # 4. Rotate Local Vector to Global Vector
        theta_rad = math.radians(direction)
        
        # Using the axis mapping: 0 deg -> (-1, 0), 90 deg -> (0, 1)
        # Standard 2D Rotation Matrix:
        # x' = x*cos(theta) - y*sin(theta)
        # y' = x*sin(theta) + y*cos(theta)
        # But adjusted for your starting offset (0 deg = -x):
        
        global_vx = (local_v[0] * -math.cos(theta_rad)) - (local_v[1] * math.sin(theta_rad))
        global_vy = (local_v[0] * math.sin(theta_rad)) + (local_v[1] * -math.cos(theta_rad))

        return (global_vx, global_vy)
