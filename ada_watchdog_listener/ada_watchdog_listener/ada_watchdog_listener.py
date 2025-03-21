# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

"""
This module defines the ADAWatchdogListener class, which listens to the watchdog
topic and can be used in two ways:
    A: Query the watchdog listener to determine if the watchdog is `ok()`.
    B: Pass in a callback function to be called when the watchdog status changes.
"""

# Standard imports
import threading
from typing import Callable, Optional

# Third-party imports
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time


# pylint: disable=too-few-public-methods
# This class only needs one public method to check if the watchdog is ok.
class ADAWatchdogListener:
    """
    The ADAWatchdogListener class listens to the watchdog topic and can be used
    in two ways:
        A: Query the watchdog listener to determine if the watchdog is `ok()`.
        B: Pass in a callback function to be called when the watchdog status
            changes.
    """

    # pylint: disable=too-many-instance-attributes
    # One extra is fine in this case.

    def __init__(
        self,
        node: Node,
        callback_fn: Optional[Callable] = None,
        sub_callback_group: Optional[CallbackGroup] = None,
        timer_callback_group: Optional[CallbackGroup] = None,
    ) -> None:
        """
        Initialize the watchdog listener.

        Parameters
        ----------
        node: the ROS node that this watchdog listener is associated with.
        callback_fn: If not None, this function will be called when the watchdog
            status changes. The function should take in a single boolean argument
            that is True if the watchdog is ok, else False.
        sub_callback_group: The callback group to use for the watchdog subscriber.
            If None, a new MutuallyExclusiveCallbackGroup will be created.
        timer_callback_group: The callback group to use for the watchdog timer.
            If None, a new MutuallyExclusiveCallbackGroup will be created.
        """
        # Store the node
        self._node = node

        # Read the watchdog_timeout_sec parameter
        watchdog_timeout_sec = self._node.declare_parameter(
            "watchdog_timeout_sec",
            0.5,  # default value
            ParameterDescriptor(
                name="watchdog_timeout_sec",
                type=ParameterType.PARAMETER_DOUBLE,
                description=(
                    "The maximum time (s) that the watchdog can go without "
                    "publishing before the watchdog fails."
                ),
                read_only=True,
            ),
        )
        self.watchdog_timeout_sec = Duration(seconds=watchdog_timeout_sec.value)

        # Read the watchdog_check_hz parameter
        watchdog_check_hz = self._node.declare_parameter(
            "watchdog_check_hz",
            60.0,  # default value
            ParameterDescriptor(
                name="watchdog_check_hz",
                type=ParameterType.PARAMETER_DOUBLE,
                description=(
                    "The rate (Hz) at which to check the whether the watchdog has failed."
                    "This parameter is only used if a callback function is passed to the"
                    "watchdog listener."
                ),
                read_only=True,
            ),
        )

        # Read the initial_wait_time_sec parameter
        initial_wait_time_sec = self._node.declare_parameter(
            "initial_wait_time_sec",
            0.0,  # default value
            ParameterDescriptor(
                name="initial_wait_time_sec",
                type=ParameterType.PARAMETER_DOUBLE,
                description=(
                    "Additional time to add to `watchdog_timeout_sec` for the first "
                    "watchdog message only. This is because even after the watchdog "
                    "subscriber is created, it takes some time to actually start "
                    "receiving messages."
                ),
                read_only=True,
            ),
        )

        # Initializing `watchdog_failed` to False lets the node wait up to `watchdog_timeout_sec`
        # sec to receive the first message
        self.watchdog_failed = False
        # Add a grace period of `initial_wait_time_sec` sec for receiving the
        # first watchdog message
        self.last_watchdog_msg_time = self._node.get_clock().now() + Duration(
            seconds=initial_wait_time_sec.value
        )
        self.watchdog_lock = threading.Lock()
        # Initialize the subscriber callback group
        if sub_callback_group is None:
            sub_callback_group = MutuallyExclusiveCallbackGroup()
        # Subscribe to the watchdog topic
        self.watchdog_sub = self._node.create_subscription(
            DiagnosticArray,
            "~/watchdog",
            self.__watchdog_callback,
            1,
            callback_group=sub_callback_group,
        )

        # If a callback function is passed in, check the watchdog at the specified rate
        if callback_fn is not None:
            self.callback_fn = callback_fn
            self._prev_status = None
            if timer_callback_group is None:
                timer_callback_group = MutuallyExclusiveCallbackGroup()
            timer_period = 1.0 / watchdog_check_hz.value
            self.timer = self._node.create_timer(
                timer_period,
                self.__timer_callback,
                callback_group=timer_callback_group,
            )

    def __watchdog_callback(self, msg: DiagnosticArray) -> None:
        """
        Callback function for the watchdog topic. This function checks if the
        watchdog has failed (i.e., if any DiagnosticStatus has a level that is
        not DiagnosticStatus.OK).

        Parameters
        ----------
        msg: The watchdog message.
        """
        watchdog_failed = False
        for status in msg.status:
            if status.level != DiagnosticStatus.OK:
                self._node.get_logger().error(
                    f"Watchdog failed: {status.message}", throttle_duration_sec=1
                )
                watchdog_failed = True
                break

        with self.watchdog_lock:
            self.watchdog_failed = watchdog_failed
            self.last_watchdog_msg_time = Time.from_msg(msg.header.stamp)

    # pylint: disable=invalid-name
    # This matches the corresponding method name in rclpy.
    def ok(self) -> bool:
        """
        Returns
        -------
        True if the watchdog is OK and has not timed out, else False.
        """
        with self.watchdog_lock:
            watchdog_failed = self.watchdog_failed
            last_watchdog_msg_time = self.last_watchdog_msg_time

        if watchdog_failed:
            return False
        time_since_last_msg = self._node.get_clock().now() - last_watchdog_msg_time
        if time_since_last_msg > self.watchdog_timeout_sec:
            self._node.get_logger().error(
                "Did not receive a watchdog message for > "
                f"{self.watchdog_timeout_sec.nanoseconds / 10.0**9} seconds! "
                f"Time since last message: {time_since_last_msg.nanoseconds / 10.0**9} seconds.",
                throttle_duration_sec=1,
            )
            return False
        return True

    def __timer_callback(self) -> None:
        """
        If the watchdog has failed, call the callback function.
        """
        # Get the watchdog status
        curr_status = self.ok()

        # Check if the watchdog status has changed since the last time this function was called
        if self._prev_status is None or curr_status != self._prev_status:
            # If it has, call the callback function
            self._node.get_logger().debug(
                f"Watchdog status (whether it is ok) changed to {curr_status}"
            )
            self.callback_fn(curr_status)

        # Update the previous status
        self._prev_status = curr_status
