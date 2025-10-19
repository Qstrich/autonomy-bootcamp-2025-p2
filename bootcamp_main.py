"""
Bootcamp F2025

Main process to setup and manage all the other working processes
"""

# pylint: disable=no-value-for-parameter,unexpected-keyword-arg

import multiprocessing as mp
import queue
import time

from pymavlink import mavutil

from modules.common.modules.logger import logger
from modules.common.modules.logger import logger_main_setup
from modules.common.modules.read_yaml import read_yaml
from modules.command import command
from modules.command import command_worker
from modules.heartbeat import heartbeat_receiver_worker
from modules.heartbeat import heartbeat_sender_worker
from modules.telemetry import telemetry_worker
from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from utilities.workers import worker_manager


# MAVLink connection
CONNECTION_STRING = "tcp:localhost:12345"

# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Set queue max sizes (<= 0 for infinity)
HEARTBEAT_QUEUE_MAX_SIZE = 10
TELEMETRY_QUEUE_MAX_SIZE = 10
REPORT_QUEUE_MAX_SIZE = 10

# Set worker counts
HEARTBEAT_SENDER_WORKER_COUNT = 1
HEARTBEAT_RECEIVER_WORKER_COUNT = 1
TELEMETRY_WORKER_COUNT = 1
COMMAND_WORKER_COUNT = 1

# Any other constants
TARGET_POSITION = command.Position(10, 20, 30)
MAIN_LOOP_DURATION = 100  # seconds

# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================


def main() -> int:
    """
    Main function.
    """
    # Configuration settings
    result, config = read_yaml.open_config(logger.CONFIG_FILE_PATH)
    if not result:
        print("ERROR: Failed to load configuration file")
        return -1

    # Get Pylance to stop complaining
    assert config is not None

    # Setup main logger
    result, main_logger, _ = logger_main_setup.setup_main_logger(config)
    if not result:
        print("ERROR: Failed to create main logger")
        return -1

    # Get Pylance to stop complaining
    assert main_logger is not None

    # Create a connection to the drone. Assume that this is safe to pass around to all processes
    # In reality, this will not work, but to simplify the bootamp, preetend it is allowed
    # To test, you will run each of your workers individually to see if they work
    # (test "drones" are provided for you test your workers)
    # NOTE: If you want to have type annotations for the connection, it is of type mavutil.mavfile
    connection = mavutil.mavlink_connection(CONNECTION_STRING)
    connection.wait_heartbeat(timeout=30)  # Wait for the "drone" to connect

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Create a worker controller
    controller = worker_controller.WorkerController()

    # Create a multiprocess manager for synchronized queues
    mp_manager = mp.Manager()

    # Create queues
    heartbeat_queue = queue_proxy_wrapper.QueueProxyWrapper(mp_manager, HEARTBEAT_QUEUE_MAX_SIZE)
    telemetry_queue = queue_proxy_wrapper.QueueProxyWrapper(mp_manager, TELEMETRY_QUEUE_MAX_SIZE)
    report_queue = queue_proxy_wrapper.QueueProxyWrapper(mp_manager, REPORT_QUEUE_MAX_SIZE)

    # Create worker properties for each worker type (what inputs it takes, how many workers)
    # Heartbeat sender
    heartbeat_sender_properties = worker_manager.WorkerProperties(
        num_workers=HEARTBEAT_SENDER_WORKER_COUNT,
        worker_function=heartbeat_sender_worker.heartbeat_sender_worker,
        worker_kwargs={
            "connection": connection,
            "controller": controller,
        },
    )

    # Heartbeat receiver
    heartbeat_receiver_properties = worker_manager.WorkerProperties(
        num_workers=HEARTBEAT_RECEIVER_WORKER_COUNT,
        worker_function=heartbeat_receiver_worker.heartbeat_receiver_worker,
        worker_kwargs={
            "connection": connection,
            "heartbeat_queue": heartbeat_queue,
            "controller": controller,
        },
    )

    # Telemetry
    telemetry_properties = worker_manager.WorkerProperties(
        num_workers=TELEMETRY_WORKER_COUNT,
        worker_function=telemetry_worker.telemetry_worker,
        worker_kwargs={
            "connection": connection,
            "telemetry_queue": telemetry_queue,
            "controller": controller,
        },
    )

    # Command
    command_properties = worker_manager.WorkerProperties(
        num_workers=COMMAND_WORKER_COUNT,
        worker_function=command_worker.command_worker,
        worker_kwargs={
            "connection": connection,
            "target": TARGET_POSITION,
            "telemetry_queue": telemetry_queue,
            "report_queue": report_queue,
            "controller": controller,
        },
    )

    # Create the workers (processes) and obtain their managers
    heartbeat_sender_manager = worker_manager.WorkerManager(heartbeat_sender_properties)
    heartbeat_receiver_manager = worker_manager.WorkerManager(heartbeat_receiver_properties)
    telemetry_manager = worker_manager.WorkerManager(telemetry_properties)
    command_manager = worker_manager.WorkerManager(command_properties)

    # Start worker processes
    heartbeat_sender_manager.start_workers()
    heartbeat_receiver_manager.start_workers()
    telemetry_manager.start_workers()
    command_manager.start_workers()

    main_logger.info("Started")

    # Main's work: read from all queues that output to main, and log any commands that we make
    # Continue running for 100 seconds or until the drone disconnects
    start_time = time.time()
    while time.time() - start_time < MAIN_LOOP_DURATION:
        # Read from heartbeat queue
        if not heartbeat_queue.queue.empty():
            try:
                heartbeat_status = heartbeat_queue.queue.get_nowait()
                main_logger.info(f"Heartbeat status: {heartbeat_status}")

                if heartbeat_status == "Disconnected":
                    main_logger.warning("Drone disconnected, exiting")
                    break
            except queue.Empty:
                pass

        # Read from report queue
        if not report_queue.queue.empty():
            try:
                report = report_queue.queue.get_nowait()
                main_logger.info(f"Command report: {report}")
            except queue.Empty:
                pass

        time.sleep(0.1)

    # Stop the processes
    controller.request_exit()

    main_logger.info("Requested exit")

    # Fill and drain queues from END TO START
    report_queue.fill_and_drain_queue()
    telemetry_queue.fill_and_drain_queue()
    heartbeat_queue.fill_and_drain_queue()

    main_logger.info("Queues cleared")

    # Clean up worker processes
    command_manager.join_workers()
    telemetry_manager.join_workers()
    heartbeat_receiver_manager.join_workers()
    heartbeat_sender_manager.join_workers()

    main_logger.info("Stopped")

    # We can reset controller in case we want to reuse it
    # Alternatively, create a new WorkerController instance

    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Failed with return code {result_main}")
    else:
        print("Success!")
