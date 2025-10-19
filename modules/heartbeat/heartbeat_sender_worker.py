"""
Heartbeat worker that sends heartbeats periodically.
"""

import os
import pathlib
import time

from pymavlink import mavutil

from utilities.workers import worker_controller
from . import heartbeat_sender
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def heartbeat_sender_worker(
    connection: mavutil.mavfile,
    # Add other necessary worker arguments here
    controller: worker_controller.WorkerController,
    heartbeat_period: float = 1.0,
) -> None:
    """
    Worker process.

    args... describe what the arguments are
    """
    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (heartbeat_sender.HeartbeatSender)
    # Create HeartbeatSender
    success, sender = heartbeat_sender.HeartbeatSender.create(connection)
    if not success:
        local_logger.error("Failed to create HeartbeatSender.")
        return

    local_logger.info("HeartbeatSender created successfully", True)
    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()
        sender.run()
        local_logger.info("Heartbeat sent", True)
        time.sleep(heartbeat_period)  # Send once per second

    local_logger.info("Worker exiting", True)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
