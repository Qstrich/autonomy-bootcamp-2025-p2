"""
Heartbeat receiving logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatReceiver:
    """
    HeartbeatReceiver class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, HeartbeatReceiver] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        return True, HeartbeatReceiver(cls.__private_key, connection, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        # Do any intializiation here
        self.connection = connection
        self.local_logger = local_logger
        self.missed_heartbeats = 0
        self.status = "Connected"
        self.DISCONNECT_THRESHOLD = 5

    def run(
        self,
    ) -> str:
        """
        Attempt to recieve a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        """
        # Try to receive a HEARTBEAT message with 1 second timeout
        msg = self.connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)

        if msg and msg.get_type() == "HEARTBEAT":
            # Received heartbeat successfully
            self.missed_heartbeats = 0
            self.status = "Connected"
            self.local_logger.info("Received heartbeat", True)
        else:
            # Missed a heartbeat
            self.missed_heartbeats += 1
            self.local_logger.warning(f"Missed heartbeat (count: {self.missed_heartbeats})", True)

            if self.missed_heartbeats >= self.DISCONNECT_THRESHOLD:
                self.status = "Disconnected"
                self.local_logger.error(
                    f"Connection lost after {self.missed_heartbeats} missed heartbeats", True
                )

        return self.status


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
