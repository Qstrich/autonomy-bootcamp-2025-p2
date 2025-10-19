"""
Decision-making logic.
"""

import math

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> "tuple[True, Command] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        return True, Command(cls.__private_key, connection, target, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        # Do any intializiation here
        self.connection = connection
        self.target = target
        self.local_logger = local_logger

        # Thresholds
        self.HEIGHT_TOLERANCE = 0.5  # meters
        self.ANGLE_TOLERANCE = math.radians(5)

        self.velocity_history = []

    def run(
        self, telemetry_data: telemetry.TelemetryData
    ) -> "tuple[True, str] | tuple[False, None]":
        """
        Make a decision based on received telemetry data.
        """

        # Log average velocity for this trip so far
        vx, vy, vz = telemetry_data.x_velocity, telemetry_data.y_velocity, telemetry_data.z_velocity
        self.velocity_history.append((vx, vy, vz))
        avg_vx = sum(v[0] for v in self.velocity_history) / len(self.velocity_history)
        avg_vy = sum(v[1] for v in self.velocity_history) / len(self.velocity_history)
        avg_vz = sum(v[2] for v in self.velocity_history) / len(self.velocity_history)
        self.local_logger.info(
            f"AVERAGE VELOCITY: ({avg_vx:.2f}, {avg_vy:.2f}, {avg_vz:.2f}) m/s", True
        )

        delta_z = self.target.z - telemetry_data.z
        if telemetry_data.z is not None and abs(delta_z) > self.HEIGHT_TOLERANCE:
            # Use COMMAND_LONG (76) message, assume the target_system=1 and target_componenet=0
            # The appropriate commands to use are instructed below
            self.connection.mav.command_long_send(
                1,  # target_system
                0,
                # Adjust height using the comand MAV_CMD_CONDITION_CHANGE_ALT (113)
                mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                0,
                1.0,  # (descent/climb rate in m/s), change from 0
                0,
                0,
                0,
                0,
                0,  # unused param
                self.target.z,  # param7 (target altitude)
            )
            # String to return to main: "CHANGE_ALTITUDE: {amount you changed it by, delta height in meters}"
            return True, f"CHANGE ALTITUDE: {delta_z:.2f}"
        # Adjust direction (yaw) using MAV_CMD_CONDITION_YAW (115). Must use relative angle to current state
        # String to return to main: "CHANGING_YAW: {degree you changed it by in range [-180, 180]}"
        # Positive angle is counter-clockwise as in a right handed system
        if telemetry_data.yaw is not None:
            dx = self.target.x - telemetry_data.x
            dy = self.target.y - telemetry_data.y
            required_yaw = math.atan2(dy, dx)
            angle_diff = (required_yaw - telemetry_data.yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_diff) > self.ANGLE_TOLERANCE:
                # Convert to degrees for command
                angle_diff_deg = math.degrees(angle_diff)
                direction = -1 if angle_diff_deg >= 0 else 1  # 1=clockwise, -1=counter-clockwise

                # Send yaw change command (relative)
                self.connection.mav.command_long_send(
                    1,  # target_system
                    0,  # target_component
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command (115)
                    0,  # confirmation
                    angle_diff_deg,  # param1 (target angle in degrees)
                    5.0,  # param2 (angular speed in deg/s) - CHANGE FROM 0
                    direction,  # param3 (direction: 1=clockwise, -1=counter-clockwise, not used for relative)
                    1,  # param4 (relative=1, absolute=0)
                    0,  # param5
                    0,  # param6
                    0,  # param7
                )

                action = f"CHANGE YAW: {angle_diff_deg:.2f}"
                # self.local_logger.info(action, True)
                return True, action

        # If no command was sent, return None explicitly
        return False, None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
