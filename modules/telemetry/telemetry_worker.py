"""
Telemtry worker that gathers GPS data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import telemetry
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def telemetry_worker(
    connection: mavutil.mavfile,
    # Add other necessary worker arguments here
    telemetry_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    connection: MAVLink connection to the drone
    telemetry_queue: Queue to send TelemetryData to Command worker
    controller: Worker controller for managing worker state
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
    # Instantiate class object (telemetry.Telemetry)
    success, telem = telemetry.Telemetry.create(connection, local_logger)
    if not success:
        local_logger.error("Failed to create Telemetry", True)
        return
    
    local_logger.info("Telemetry created", True)
    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()
        success, telemetry_data = telem.run()

        if success: #Success
            telemetry_queue.queue.put(telemetry_data)
            local_logger.info(f"Sent telemetry data: {telemetry_data}", True)
        else: # Timeout occurred, restart and try again
            local_logger.warning("Telemetry timeout, restarting", True)

# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
