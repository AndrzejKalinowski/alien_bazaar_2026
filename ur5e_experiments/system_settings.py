"""Supervisor timing defaults, in seconds; no robot motion parameters.

These deadlines are for the offline supervisor milestone. Real hardware
integration must validate them on the target computer before enabling motion.
"""

TICK_PERIOD = 0.02             # s, nominal supervisor period
TELEMETRY_MAX_AGE = 0.5        # s, oldest contributing device sample
SCENE_MAX_AGE = 0.5            # s, latest observation before a pick
FUTURE_TOLERANCE = 0.05        # s, reject invalid timestamps
PICK_TIMEOUT = 30.0           # s, travel from OBSERVE at 0.15/0.05 m/s + contact move
                              # + grip confirmation (6 s); a deadline, not a motion limit
MOTION_TIMEOUT = 30.0         # s
STATION_TIMEOUT = 30.0        # s
RELEASE_TIMEOUT = 3.0         # s, firmware release pulse is 1.5 s
STOP_TIMEOUT = 2.0            # s, deadline for acknowledged stop
SIM_OPERATION_TIME = 0.10     # s, artificial duration (not a hardware setting)
MAX_START_REQUESTS = 256      # retained IDs per supervisor instance
MAX_PENDING_EVENTS = 4096     # drain from the runner after each tick
