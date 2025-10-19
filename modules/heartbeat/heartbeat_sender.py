"""
Heartbeat sending logic.
"""

from pymavlink import mavutil


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatSender:
    """
    HeartbeatSender class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
    ) -> "tuple[True, HeartbeatSender] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatSender object.
        """
        return True, HeartbeatSender(cls.__private_key, connection)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
    ) -> None:
        assert key is HeartbeatSender.__private_key, "Use create() method"

        # Do any intializiation here
        self.connection = connection

    def run(
        self,
    ) -> str:
        """
        Attempt to send a heartbeat message.
        """
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,  # base_mode
            0,  # custom_mode
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

        return f"HeartbeatSender(connection={self.connection})"


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
