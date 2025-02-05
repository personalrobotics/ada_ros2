# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

"""
This package contains the ADAWatchdogListener class and a node that kills itself
when the watchdog fails. The latter is useful to shutdown an entire launchfile
(e.g., the one that launched the controllers) if the watchdog fails.
"""

from .ada_watchdog_listener import ADAWatchdogListener
