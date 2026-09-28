"""
Pick an upside-down glass found by the overhead camera with the suction cup
and put it down on an AprilTag lying on the table.

Uses everything from find_glasses.py (camera calibration, detection area,
colour filter, trackbars); the glasses stand upside down, so the circle seen
from above is the foot. GLASS_HEIGHT (table to top of the foot) is both the
height of that circle for the detection and where the suction cup grabs; it is
separate from find_glasses.RIM_HEIGHT, so tuning one script can't break the other.

Task (p / gamepad Y):
  0. With GO_TO_START_FIRST, away from the start position (START_Q, the same
     as h), moveJ there first; the rest starts when it has arrived.
  1. Measure the glasses and the place tag (averaged over several frames; the
     robot at START_Q is out of the camera's view).
  2. Choose the glass closest to the robot base (glasses standing on a place
     tag are skipped) and the next free place tag (see below).
  3. A wrist wound up more than UNWIND_ABOVE_DEG (jogging and moveL wind
     them up a turn at a time) is turned back a full turn first, gripper
     empty, where the arm stands (raised if the swing would sag), one wrist
     at a time: wrist 2, then wrist 1 with the tool parked pointing out
     along the wrist-1 axis, then wrist 3 (both at once swung the gripper
     through the forearm). Then the steps of SEQUENCE, one after the other.
  Every joint move (tool turns, unwinding, spray moves) is first checked in
  the system_arm capsule model: the gripper may not come closer to the arm
  than in the closest taught pose (SELF_GAP_MARGIN), so a move the pendant
  would stop with "tool hitting the arm" is refused before it is sent.
  Exclusion zones (EXCLUSION_ZONES, boxes in the base frame, drawn red in the
  video): no move may take the TCP into one, grown by EXCLUSION_MARGIN and,
  with a glass held, by a sphere holding the whole glass. Checked before the
  move is sent (moveL as a line, moveJ along its arc in the system_arm model,
  pushes, unwinding, h), and refused if it would enter. The station moves
  (to, into, out of and between spray / sponge / dry) are exempt: the devices
  are in there. A move that starts inside a zone may leave it. Only the TCP and
  the glass are checked, not the arm links. Teach a box with b at two opposite
  corners.
  4. Up, the tool turns back to its start orientation, back to the start pose
     (keep it out of the camera view).
  SEQUENCE is checked before anything moves (and at program start): a pick with
  a glass held, a flip without a side grip, a put-down with nothing held or a
  glass still held at the end is refused. Every step starts and ends with the
  tool at carry height (clears the other glasses by CARRY_CLEARANCE, with what
  it holds, and never below TRAVERSE_HEIGHT above the table: travel, turns,
  flips and unwinding happen above that plane; station moves go below it).
  Whenever a step needs another tool orientation (sideways for pick_side,
  pointing down for pick_top), the tool goes up to carry height and
  turns where it is (TURN_OVER_START: over the start position instead, away
  from the glasses, slower), so the start orientation can be anything. The
  turn is a joint move to the IK solution nearest the taught
  joints (SIDE_GRIP_Q / TOP_GRIP_Q, o prints them) or else the joints at the
  start of the task, so the arm has the taught shape and the joints end the
  same way every cycle (moveL turns let a wrist wind up a turn per cycle until
  "joint close to limits"). Between two arm shapes the TCP sags on the joint
  arc; the sag is predicted and the turn done that much higher (up to the
  ceiling), so it never goes more than TURN_MAX_DIP below the safe height.
  After every grip, the glass position is taken from
  where the cup is, not the camera. The window shows "step i/n name: what it does".
  The default SEQUENCE flips the glass, sprays, sponges and dries it and places
  it: pick_side, flip, set_down, pick_top, spray, sponge, dry, place. To add a step, write a step_<name> generator in SequenceTask and give
  it a STEP_GRIPS entry (what it needs held before, what it leaves held).

Steps:
  pick_top   Over the glass, down to APPROACH_GAP above its top (GLASS_HEIGHT),
             vacuum on, slowly down until the force sensor feels contact, wait
             for "GRIP OK", lift. The cup goes TOP_GRIP_OFFSET (base x / y)
             from the glass axis, also when placing. The orientation is taught: TOP_GRIP_ROTATION
             (jog the tool pointing down, turned as it should grip, press o).
             With None: the start orientation if it points down (within
             MAX_TILT_DEG), else the one at START_Q.
  pick_side  The suction cup grabs the glass wall at SIDE_GRIP_HEIGHT above the
             table with the tool horizontal: to SIDE_STANDOFF in front of the
             wall at carry height, down to SIDE_GRIP_HEIGHT, vacuum on, slowly
             sideways into the wall. A glass slides at ~1 N, long before the
             force sensor notices, so this stops at the expected wall +
             SIDE_MAX_PRESS (the force limit only guards against hitting
             something solid). Wait for "GRIP OK", lift.
  flip       Up to where the turning glass clears the others, turn wrist 3 by
             180 deg (moveJ, FLIP_SPEED): the tool z axis is horizontal and
             crosses the glass axis, so the glass turns upside down over the
             same spot and the TCP stays put. Needs a TCP offset without x/y
             (MAX_TCP_XY_OFFSET). The next tool turn brings wrist 3 back, so
             the cable and hose don't wind up. Watches for
             "GRIP LOST" like every move with the glass held.
  set_down   Lower the held glass where it is (the spot it was picked from is
             free), slowly down until it touches the table, release, up (a
             side grip backs off SIDE_STANDOFF sideways first).
  spray      Joint move (glass held, arc checked) to SPRAY_POSE + SPRAY_APPROACH
             (5 cm back along base x), in the taught arm configuration SPRAY_Q;
             straight into SPRAY_POSE; SPRAY_STROKES pump strokes, one every
             SPRAY_PERIOD, of the sprayer servo (bus_servos.Sprayer on
             SPRAYER_PORT, in its own thread, so the loop goes on); back out the
             same way and straight up to the traverse plane (every station
             leaves upwards). Refused before anything moves when the sprayer
             did not answer at program start. Stopping (s, sticks, a fault)
             ends the spraying after the current stroke; the glass stays held.
  sponge     Like spray at the taught SPONGE_POSE, coming from above
             (SPONGE_APPROACH), then the sponge servo (bus_servos.ROTATOR_ID)
             turns SPONGE_SPIN_TIME one way at SPONGE_SPEED and as long the
             other way, then stops (also on s / abort / failure).
  dry        Joint move (glass held, arc checked) to DRY_APPROACH back along the
             tool z axis from the taught DRY_POSE (tool pointing down: from
             above), in the taught configuration DRY_Q; along the tool z axis
             into DRY_POSE (DRY_DEPTH deeper); wrist 3 turns to each angle of
             DRY_TURNS_DEG (now +180 deg once; about the tool z axis: the TCP
             stays, the glass turns about its own axis with the top grip; needs
             the TCP on the flange axis like the flip); out along the tool z
             axis as turned.
             spray, sponge and dry are stations: from one straight to the next
             when that move passes the checks, otherwise back through the
             joints the arm had before the first one; always back there before
             any other step.
             With the glass held, the joint arcs may sag TURN_MAX_DIP below the
             lower of carry height and the move's two ends.
  place      The same as set_down, with the glass axis over the place tag.
  The flipped glass is GLASS_HEIGHT tall either way up, so pick_top after a
  flip lands at the same height, on the end that stood on the table before
  (for an upside-down glass: the rim).

Side grip: the grip orientation is taught: jog the cup onto a glass wall exactly as it
  should grip (hold RB for rotation) and press o. It prints SIDE_GRIP_ROTATION
  (the tool orientation in the base frame, used as it is for every glass, the
  approach runs along its tool z axis) and SIDE_GRIP_HEIGHT; copy them into
  the constants. With SIDE_GRIP_ROTATION = None the tool is held level instead,
  set by SIDE_APPROACH_YAW_DEG (relative to the base -> glass direction) and
  SIDE_ROLL_DEG (the turn around the tool's own axis: 0 = tool x axis pointing
  straight down, positive = right-hand around the approach direction); o
  prints those too when the tool is roughly level.
  The wall radius at the grip height is SIDE_GRIP_RADIUS, or half the measured
  foot diameter for straight-sided glasses. The approach path is not checked:
  the gripper must fit between the glasses on the robot side of the target.

Keys (video window) / gamepad:
  p  / Y (north)   run SEQUENCE on every glass, one after another (PICK_ALL;
                   False: one glass per p): after each one, back at the start
                   pose, measure again and do the next, until no glass or no
                   free place tag is left or one fails; s / sticks stop it.
                   With GO_TO_START_FIRST the arm goes to START_Q first
  s  / A (south)   stop / abort (vacuum stays as it is)
  g  / B (east)    grip (vacuum on)
  r  / X (west)    release
  y  / Back        spray MANUAL_SPRAY_STROKES strokes; again while spraying: stop
  b  / left stick click    teach an exclusion box: press at two opposite corners
                   (tip position), the EXCLUSION_ZONES line is printed
  f  / right stick click   freedrive on / off (move the arm by hand; the sticks
                   don't jog, p / h are refused until it is off; s, an error
                   or quitting also end it). Not limited by MAX_TCP_Z. Handy
                   with o to teach poses.
                   (refused while a task runs or without the sprayer)
  h  / Start       go to the start position (START_Q, tool pointing down)
  o                print the taught constants of the current tool pose: TOP_GRIP_*
                   when it points down, else SIDE_GRIP_* and SPRAY_POSE / SPRAY_Q
  c                clear the used place tags (a new round; keyboard only)
  q / Esc          quit
  Sticks jog the robot (gamepad_jog.py); touching them aborts a running task.
  The mouse edits the detection area as in find_glasses.py.
  A command that can't run right now (g during the 1.5 s release pulse or
  while a task runs, p / h while a task runs, ...) is refused with a WARNING
  in the console and the window; the program keeps running.

All motion goes through safe_motion.py: the TCP never goes above MAX_TCP_Z
(0.85 m above the base). A move that would is refused and the task stops.

Several glasses, several place tags (any 36h11 size, flat on the table in the
camera view): every p takes the glass closest to the robot base (horizontal
distance from the base axis) and puts it on the next free tag: the lowest id
(or the next in PLACE_TAG_IDS) that has not had a glass put on it this round
and has no glass standing on it (PLACED_RADIUS). Glasses standing on a tag
are never picked. Each tag's last seen position is kept, since the glass put
on it covers it. A tag counts as used once its glass is put down (a task that
fails before that leaves it free). All used: p is refused; c clears the used
tags for the next round. In the video the next tag is a bright cyan square,
used ones gray, the others dark cyan. With PLACE_TAG_IDS = None every tag in
view is a place tag, so take calibration tags off the table.

Motion watchdog (robot_watchdog.py): the robot stops by itself if the main
loop sends nothing for 0.2 s (stalled loop, camera hang, breakpoint). The
next loop iteration then re-uploads the control script and aborts the task.

Loop timing: the camera is read in its own thread (FrameGrabber), and glasses
and tags are detected only between tasks, so during a task the robot is
stepped every LOOP_PERIOD (10 ms) instead of every video frame (it was 10 Hz:
moves ended late and the force-guarded pushes stuttered). The video and
detection run on new frames only. A move is done when the controller's async
change count has moved on since the command and nothing runs any more (no
fixed start-up wait), so the next move follows within a loop step.

Faults are recovered in place, without restarting (which would also reset the
gripper, 2 s): any error in the loop (a robot call failing after a protective
stop, a lost RTDE connection, no camera frame for FRAME_TIMEOUT) stops all
motion, aborts the task, prints the traceback and a WARNING, reconnects RTDE
if needed and carries on. After a protective stop, clear it on the pendant;
the control script is then re-uploaded by itself. Ctrl+C, q / Esc or closing
the window still quit.

Requires: pip install opencv-python ur_rtde pyserial numpy (bus_servos.py for spray)
"""

import threading
import time
import traceback
from itertools import chain

import cv2
import numpy as np

import bus_servos
import find_glasses as fg
import safe_motion
import system_arm
from follow_april_tag import IP, MAX_TCP_Z, MIN_TCP_Z, connect_suction
from gamepad_jog import GamepadControl, Jogger
from robot_watchdog import RobotWatchdog
from safe_motion import MotionRefused
from serial import SerialException
from suction import key_command

PLACE_TAG_IDS = None         # place tags in the order they are filled; None = every tag in view, by id
PLACED_RADIUS = 0.04         # m, a glass this close to a place tag already stands on it
GLASS_HEIGHT = 0.075         # m, upside-down glass: table to top of the foot (seen circle, cup lands here)

CARRY_CLEARANCE = 0.05       # m, gap under the carried glass over the other glasses
TRAVERSE_HEIGHT = 0.30       # m above the table: the TCP travels, turns, flips and unwinds at least
                             # this high (station moves go down to the devices)
APPROACH_GAP = 0.02          # m, stop this far above the foot / placing height, then go slowly
MAX_OVERSHOOT = 0.015        # m, push at most this far past the expected height (placing)
TOP_PICK_OVERSHOOT = 0.025   # m, the same for the top pick (was MAX_OVERSHOOT, 15 mm); the push still
                             # stops at CONTACT_FORCE on the glass
MAX_TILT_DEG = 10            # deg, "pointing down" / "horizontal" within this
TURN_TOLERANCE_DEG = 1       # deg, a smaller orientation change needs no turn
TURN_OVER_START = False      # True: tool turns go back over the start position (away from the glasses);
                             # False: where the tool is, at carry height (saves ~3 round trips a cycle)

# --- sequence -------------------------------------------------------------------
# What p does with the chosen glass, step by step (SequenceTask.step_<name>):
#   pick_top   grab the top of the glass, tool pointing down
#   pick_side  grab the glass wall, tool horizontal (side grip below)
#   flip       turn the side-gripped glass upside down in the air (wrist 3, 180 deg)
#   set_down   put the held glass down where it is and release
#   spray      hold the glass in the taught SPRAY_POSE and work the sprayer servo
#   sponge     hold the glass in the taught SPONGE_POSE and spin the sponge servo
#   dry        hold the glass in the taught DRY_POSE and swing it about the tool axis
#   place      put the held glass down on the place tag and release
# ["pick_side", "place"] / ["pick_top", "place"] are the plain pick & place.
SEQUENCE = ["pick_side", "flip", "set_down", "pick_top", "spray", "sponge", "dry", "place"]

# --- flip -----------------------------------------------------------------------
FLIP_SPEED = 1.2             # rad/s, wrist 3 turn with the glass held (was 0.5, 0.9)
FLIP_ACCEL = 1.2             # rad/s^2 (was 0.5, 0.9)
MAX_TCP_XY_OFFSET = 0.005    # m, the flip turns around the flange axis: a TCP off it would swing the glass

# --- tool turns (see SequenceTask.turn) ------------------------------------------------
TURN_SPEED = 1.5             # rad/s, moveJ to the turned orientation (nothing held; was 0.5, 1.0)
TURN_ACCEL = 1.5             # rad/s^2 (was 0.5, 1.0)
TURN_MAX_DIP = 0.02          # m, the TCP may sag this far below the safe height on the joint arc;
                             # a turn that sags more is done higher up (see SequenceTask.turn)
TURN_RAISE_MARGIN = 0.01     # m, extra height each time the turn is raised
TURN_RAISE_TRIES = 3         # re-plans at a raised height before refusing
TURN_IK_TOLERANCE = 0.002    # m, IK solution checked against the target with forward kinematics
TURN_CHECK_STEPS = 20        # forward-kinematics samples along the turn
CONFIG_TOLERANCE_DEG = 5     # deg, joints this close to the IK solution near the taught ones = the
                             # taught arm configuration (another branch is ~180 deg off)
IK_MAX_ERROR = 1e-10         # m / rad, getInverseKinematics max position / orientation error (its default)

# --- exclusion zones ----------------------------------------------------------------
# Boxes the tool may not enter while traversing (carry, turns, flips, pushes, unwinding,
# h), in the base frame: ([xmin, ymin, zmin], [xmax, ymax, zmax]) in m. The station moves
# (spray, sponge, dry: to, into, out of and between them) may, the devices are there.
# Teach one with b / left stick click at two opposite corners (it prints the line).
# EXCLUSION_ZONES = [([0.086, -0.767, 0.054], [0.292, -0.431, 0.274])]
EXCLUSION_ZONES =[]
EXCLUSION_MARGIN = 0.03      # m around the TCP (the gripper body); with a glass held, plus a
                             # sphere around the TCP that holds the whole glass

# --- spray ------------------------------------------------------------------------
# Taught spray pose, TCP in the base frame (o prints it): the glass held from the top,
# the tool (and glass) pointing along base +x, 27 deg up, in front of the sprayer.
# SPRAY_Q = the arm configuration there (joints in rad, wrapped to +-180 deg). The arm
# goes to SPRAY_POSE + SPRAY_APPROACH with a joint move, then straight in, sprays, and
# comes back the same way.
SPRAY_POSE = [0.02906, -0.48694, 0.14569, 1.26169, 0.53405, 2.03612]
SPRAY_Q = [-1.55987, -2.38400, -2.13538, -0.22390, -2.67745, 2.29890]
SPRAY_APPROACH = [-0.05, 0.0, 0.0]  # m in the base frame, start of the straight last bit: along
                                    # +x into the spray pose (the glass points +x there). Further
                                    # back folds the arm tighter (model: -15 mm at 5 cm, -24 at 10)
SPRAY_MOVE_SPEED = 1.2       # rad/s, joint moves with the glass held (as FLIP_SPEED; was 0.5, 0.9)
SPRAY_MOVE_ACCEL = 1.2       # rad/s^2 (was 0.5, 0.9)
SPRAY_STROKES = 2            # pump strokes (was 6, 10, 5, 3)
SPRAY_PERIOD = 1.2           # s between the starts of two strokes (bus_servos default 2.0; a stroke
                             # itself takes ~0.5 s: press, SPRAY_HOLD 0.2 s, release)
SPRAY_TIMEOUT = SPRAY_STROKES * SPRAY_PERIOD + 8.0   # s, the strokes must be done by then
SPRAYER_PORT = bus_servos.PORT   # COM port of the bus servo board (machine-specific)
MANUAL_SPRAY_STROKES = 3     # pump strokes per y / Back press (at SPRAY_PERIOD)

# --- sponge -------------------------------------------------------------------------
# Taught sponge pose (o prints it like SPRAY_POSE): the glass held from the top, tool
# pointing down (6 deg off), on the sponge. SPONGE_Q = the arm configuration there. The
# arm goes (joint move) to SPONGE_APPROACH above it, straight down onto it, then the
# sponge servo (bus_servos.ROTATOR_ID, same board as the sprayer) turns SPONGE_SPIN_TIME
# one way and SPONGE_SPIN_TIME the other, then stops.
SPONGE_POSE = [0.29664, -0.60054, 0.18340, -2.09057, 2.28208, 0.14519]
SPONGE_Q = [-0.91937, -2.10711, -1.45363, -1.24989, 1.60516, -0.83510]
SPONGE_APPROACH = [0.0, 0.0, 0.10]  # m in the base frame: from above, straight down onto it
SPONGE_SPEED = 2890          # steps/s, 85 % of the ST3215's ~3400 (4096 steps a turn)
SPONGE_SPIN_TIME = 10.0      # s each way

# --- dry --------------------------------------------------------------------------
# Taught drying pose (o prints it like SPRAY_POSE), glass held from the top, tool pointing
# down (2 deg off). DRY_Q = the arm configuration there. The arm goes (joint move) to
# DRY_APPROACH back along the tool z axis from it (here: above it), moves in along the tool
# z axis, then turns wrist 3 (= about the tool z axis, the glass axis with the top grip)
# to each of DRY_TURNS_DEG in turn, and backs out along the tool z axis as it is turned.
DRY_POSE = [0.18048, -0.71972, 0.16708, -3.09267, 0.34858, 0.02415]
DRY_Q = [-1.14279, -2.26088, -1.25626, -1.22920, 1.57242, -2.49099]
DRY_APPROACH = 0.10          # m back along the tool z axis, start of the straight last bit
DRY_DEPTH = 0.0              # m further along the tool z axis than the taught DRY_POSE
DRY_TURNS_DEG = [180]        # deg, wrist 3 angles visited in order, relative to where it arrives
                             # (+ = the joint's plus direction); [60, -60] * 3 + [0] swings instead
DRY_SPEED = 2.0              # rad/s, wrist 3 turns (the glass turns about its own axis; was 1.5)
DRY_ACCEL = 3.0              # rad/s^2

# --- start ----------------------------------------------------------------------
# Start / home joints (rad): h goes here, p goes here first, the task ends here. Taught:
# TCP at 80, -506, 504 mm, tool tilted 42 deg from pointing down.
# Wrist 1 stored unwound (taught at 303 deg = -57 deg, one turn wound up).
START_Q = [-0.98655, -1.34004, -1.79234, -0.99111, 1.10915, -1.47313]
START_TOLERANCE_DEG = 1      # deg, every joint this close to START_Q counts as at the start
GO_TO_START_FIRST = False    # p moves to START_Q first when away from it (False: starts where the arm is)
PICK_ALL = True              # p does every glass, one after another (False: one glass per p)
JOINT_LIMIT_MARGIN_DEG = 60  # deg, turn targets stay this far from the +-360 deg joint limits
UNWIND_ABOVE_DEG = 200       # deg, a wrist further than this from zero is turned back a full
                             # turn at the start of a task (see SequenceTask.unwind)
SELF_GAP_MARGIN = 0.005      # m, joint moves may not bring the gripper closer to the arm than the
                             # closest taught pose does, minus this (system_arm capsules, see self_gap)
SELF_GAP_PROVEN = -0.020     # m, model gap of a pose the arm reached by hand without touching (the
                             # first spray pose): the limit is never tighter than this minus the margin
SELF_CHECK_STEP_DEG = 3      # deg of the biggest joint change between self-collision samples
UNWIND_PARK_STEP_DEG = 15    # deg, wrist-1 angles tried to make the wrist-2 turn clear
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist 1", "wrist 2", "wrist 3")

# --- top grip -------------------------------------------------------------------
# Taught pick_top orientation, axis-angle in the base frame (o key prints it): tool
# pointing down (2.3 deg off), tool x at -48 deg in the base x/y plane. The descent
# is along base -z, which is within 2.3 deg of the tool z axis. None = the start
# orientation if it points down, else the START_Q one.
TOP_GRIP_ROTATION = [2.84810, -1.26678, 0.05080]
# Taught arm configuration for it, joints in rad (o prints them): the turn to pointing
# down picks the joint solution nearest these, so the elbow and wrist are the way they
# were taught. None = nearest the joints at the start of the task.
TOP_GRIP_Q = [-1.64260, -2.27068, -1.15258, -1.25635, 1.59322, -2.37796]
# m, base x / y: where the cup goes relative to the glass axis for the top grip (the cup
# landed a bit off centre). Also used when placing, so the glass, not the cup, ends up
# on the tag.
TOP_GRIP_OFFSET = [0.0, -0.010]

# --- side grip ------------------------------------------------------------------
SIDE_GRIP_HEIGHT = 0.035    # m above the table, cup center (TCP) on the wall (taught pose); the gripper must clear the table here
SIDE_GRIP_RADIUS = None      # m, glass radius at SIDE_GRIP_HEIGHT, None = measured foot diameter / 2
# Taught grip orientation, axis-angle in the base frame (o key prints it), used for
# every glass; the approach runs along its tool z axis (here base -y, 1.8 deg down).
# Re-taught with the gripper rolled 180 deg around the approach (the wrist the other
# way round), which clears the table beside the glass.
# None = level tool built from SIDE_APPROACH_YAW_DEG / SIDE_ROLL_DEG instead.
SIDE_GRIP_ROTATION = [0.72813, 1.77884, -1.70354]
# Taught arm configuration for it, joints in rad (o prints them, jogged onto a glass as
# for SIDE_GRIP_ROTATION): the turn sideways picks the joint solution nearest these.
# Another solution for the same orientation can have the wrist or elbow the other way
# round and hit the table beside the glass. None = nearest the start joints.
SIDE_GRIP_Q = [-1.54158, -2.26731, -1.93571, -2.10989, -1.57190, -2.33435]
SIDE_APPROACH_YAW_DEG = 8    # deg, turn the approach from radial (base -> glass) around vertical
SIDE_ROLL_DEG = 138          # deg, tool turned around its own axis: 0 = tool x straight down (o key reads it off)
SIDE_STANDOFF = 0.03         # m, gap between cup and wall before the slow approach and after release
SIDE_MAX_PRESS = 0.015       # m, go at most this far past the expected wall (camera error, cup compression; was 6 mm)
SIDE_CONTACT_FORCE = 20.0     # N, stop the sideways approach early (a free glass slides before this)

MOVE_SPEED = 0.50            # m/s, moveL (was 0.15, 0.25, 0.35)
APPROACH_SPEED = 0.15        # m/s, moveL over to the glass and down next to it / onto the tag (was 0.05, 0.10)
MOVE_ACCEL = 1.0             # m/s^2 (keep low enough for the vacuum to hold the glass; was 0.6, 0.8)
DESCEND_SPEED = 0.02         # m/s, slow final approach, stopped by the force sensor (was 0.015)
DESCEND_ACCEL = 0.2
CONTACT_FORCE = 10.0         # N, touching the glass foot
PLACE_FORCE = 8.0            # N, glass touching the table
FT_SETTLE = 0.2              # s, wait after zeroing the force sensor
GRIP_DWELL = 0.5             # s, let the vacuum build up before lifting
GRIP_CONFIRM_TIMEOUT = 6.0   # s
RELEASE_TIME = 1.7           # s, the release pulse lasts 1.5 s
SPEED_CMD_TIME = 0.02        # s
MOVE_START_TIMEOUT = 1.0     # s, a move the controller never reports as started counts as done then
MOVE_SETTLE = 0.03           # s after a move ends before the next command (its robot-script thread exits)
ERROR_RETRY_DELAY = 0.1      # s, the loop retries at most this often after an error (no 100 Hz spam)
LOOP_PERIOD = 0.01           # s, shortest main loop step (the camera has its own thread)
STOP_DECEL = 1.0             # m/s^2
HOME_SPEED = 1.0             # rad/s, moveJ to START_Q
HOME_ACCEL = 1.0             # rad/s^2
RECONNECT_DELAY = 1.0        # s, wait after a failed reconnect before the loop retries

GAMEPAD_KEYS = {"BTN_NORTH": "p", "BTN_SOUTH": "s", "BTN_EAST": "g",
                "BTN_WEST": "r", "BTN_START": "h", "BTN_SELECT": "y",
                "BTN_THUMBR": "f", "BTN_THUMBL": "b"}


class TaskFailed(Exception):
    pass


def warn(message):
    """A refused operator command: printed and shown in the window, the loop goes on."""
    print(f"WARNING: {message}")
    return message


class Task:
    """Runs a generator one step per video frame; each step returns quickly."""

    def __init__(self, r, c):
        self.r = r
        self.c = c
        self.status = ""
        self.done = False
        self._steps = self.run()

    def run(self):
        yield from ()

    def update(self):
        try:
            self.status = next(self._steps)
        except StopIteration:
            self.done = True
        except (TaskFailed, MotionRefused) as e:
            print(e)
            self.stop_motion()
            self.status = str(e)
            self.done = True
        except (SerialException, OSError) as e:
            # Gripper unplugged mid-task: stop this task, not the whole program
            self.stop_motion()
            self.status = warn(f"gripper not responding ({e}), task aborted")
            self.done = True

    def abort(self):
        self._steps.close()
        self.stop_motion()

    def stop_motion(self):
        """Stop everything, the async move thread first. In ur_rtde's robot script only
        stopL / stopJ (and a new async move) kill the move thread; speedStop / speedL
        sent while it still runs make the controller stop the program with "another
        thread is already controlling the robot"."""
        self.c.stopL(STOP_DECEL)
        self.c.stopJ(STOP_DECEL)
        self.c.speedStop(STOP_DECEL)

    def async_count(self):
        """The controller's async operation change count (None without that call)."""
        try:
            return self.c.getAsyncOperationProgressEx().changeCount()
        except AttributeError:
            return None

    def move_l(self, pose, speed, accel):
        self._count_before_move = self.async_count()
        self.c.moveL(pose, speed, accel, True)

    def move_j(self, q, speed, accel):
        self._count_before_move = self.async_count()
        self.c.moveJ(q, speed, accel, True)

    def wait_move(self, label):
        """Wait for the asynchronous move started just before (move_l / move_j).

        Done once the controller's async status has changed since before the command
        and nothing runs any more, so the next move follows within a loop step (the
        old fixed 0.2 s start-up wait cost ~10 s a cycle). A move the controller never
        reports (one to where the arm already is) ends after MOVE_START_TIMEOUT."""
        start = time.time()
        before = getattr(self, "_count_before_move", None)
        while True:
            if before is None:
                # No change count: give the move a moment to start before checking
                if time.time() - start > 0.2 and self.c.getAsyncOperationProgress() < 0:
                    return
            else:
                status = self.c.getAsyncOperationProgressEx()
                if not status.isAsyncOperationRunning() and (
                        status.changeCount() != before or time.time() - start > MOVE_START_TIMEOUT):
                    break
            yield label
        # The robot script's move thread reports "finished" just before it exits: a
        # speedL / speedStop inside that gap is "another thread is already controlling
        # the robot" (the next async move kills the thread itself, the rest does not)
        settle = time.time() + MOVE_SETTLE
        while time.time() < settle:
            yield label

    def wait(self, seconds, label):
        end = time.time() + seconds
        while time.time() < end:
            yield label


def joint_near_limit(q):
    """(name, deg) of the first joint within JOINT_LIMIT_MARGIN_DEG of +-360 deg, or None.

    Cartesian moves and jogging let the joints wind up turn by turn; a task started
    there stops halfway with "joint close to limits" on the pendant."""
    for name, angle in zip(JOINT_NAMES, np.degrees(q)):
        if abs(angle) > 360 - JOINT_LIMIT_MARGIN_DEG:
            return name, angle
    return None


def at_start(r):
    """True when the joints are at START_Q (joint angles, so a wound-up wrist is not)."""
    return max(abs(a - b) for a, b in zip(r.getActualQ(), START_Q)) < np.radians(START_TOLERANCE_DEG)


class HomeTask(Task):
    """moveJ to START_Q. then_pick: p was pressed away from the start, pick when there."""

    def __init__(self, r, c, then_pick=False):
        self.then_pick = then_pick
        self.arrived = False
        super().__init__(r, c)

    def run(self):
        home_z = self.c.getForwardKinematics(START_Q)[2]
        if home_z > MAX_TCP_Z:
            raise TaskFailed(f"start is above the ceiling ({home_z:.3f} > {MAX_TCP_Z:.3f} m), not moving")
        zone = joint_path_zone_hit(self.c, [self.r.getActualQ(), START_Q], EXCLUSION_MARGIN)
        if zone is not None:
            raise TaskFailed(f"going to the start would enter exclusion zone {zone}, jog around it first")
        self.move_j(START_Q, HOME_SPEED, HOME_ACCEL)
        yield from self.wait_move("going to start..." + (" (then pick)" if self.then_pick else ""))
        self.arrived = True
        self.status = "at start"

    def abort(self):
        self._steps.close()
        self.c.stopJ(STOP_DECEL)


class FrameGrabber:
    """Reads the camera in its own thread, so the robot loop never waits for a frame.

    A read blocks ~60 ms (17 fps) and glass + tag detection take ~40 ms: with both in
    the loop it ran at 10 Hz, every move ended up to 0.1 s late and a force-guarded
    push (speedL for SPEED_CMD_TIME, 20 ms) moved only 20 ms out of every 100.
    """

    def __init__(self, cap):
        self.cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._count = 0
        self._time = time.time()
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop:
            ok, frame = self.cap.read()
            if ok:
                with self._lock:
                    self._frame, self._count, self._time = frame, self._count + 1, time.time()
            else:
                time.sleep(0.005)

    def latest(self):
        """(frame number, frame) of the newest frame; RuntimeError when the camera has
        sent nothing for fg.FRAME_TIMEOUT (the loop then stops the robot and recovers)."""
        with self._lock:
            count, frame, stamp = self._count, self._frame, self._time
        if frame is None or time.time() - stamp > fg.FRAME_TIMEOUT:
            raise RuntimeError(f"no frame from the camera for {fg.FRAME_TIMEOUT} s")
        return count, frame

    def read(self):
        """cv2.VideoCapture.read() for fg.measure(): waits for the next new frame."""
        with self._lock:
            count = self._count
        deadline = time.time() + fg.FRAME_TIMEOUT
        while time.time() < deadline:
            with self._lock:
                if self._count != count:
                    return True, self._frame
            time.sleep(0.002)
        return False, None

    def close(self):
        self._stop = True
        self._thread.join(timeout=1.0)
        self.cap.release()


def recover(r, c, watchdog):
    """After an exception in the main loop: stop all motion, reconnect what dropped out.

    The control script itself is re-uploaded by watchdog.kick() on the next
    frame, once the robot is not protective- or emergency-stopped any more.
    """
    # stopL / stopJ first: they end the robot script's async move thread, speedStop does not
    for stop in (c.stopL, c.stopJ, c.speedStop):
        try:
            stop(STOP_DECEL)
        except Exception:
            pass
    try:
        if not r.isConnected():
            print("RTDE receive connection lost, reconnecting")
            r.reconnect()
        if not c.isConnected():
            print("RTDE control connection lost, reconnecting")
            c.reconnect()
            watchdog.arm()
    except Exception as e:
        print(f"Reconnect failed ({e}), retrying on the next error")
        time.sleep(RECONNECT_DELAY)


def side_rotation(direction, roll_deg):
    """Axis-angle rotation with the tool z axis along the horizontal direction and the
    tool x axis turned roll_deg (right-hand around tool z) away from straight down."""
    z = np.array([direction[0], direction[1], 0.0])
    z /= np.linalg.norm(z)
    down = np.array([0.0, 0.0, -1.0])
    roll = np.radians(roll_deg)
    x = np.cos(roll) * down + np.sin(roll) * np.cross(z, down)
    R = np.column_stack([x, np.cross(z, x), z])
    return list(cv2.Rodrigues(R)[0].ravel())


def read_side_orientation(tcp):
    """SIDE_APPROACH_YAW_DEG and SIDE_ROLL_DEG of a jogged pose, as side_rotation() defines
    them (yaw relative to the radial direction at the TCP), or None if the tool is not
    roughly horizontal."""
    R = cv2.Rodrigues(np.asarray(tcp[3:], dtype=float))[0]
    z = R[:, 2]
    if abs(z[2]) > np.sin(np.radians(MAX_TILT_DEG)) or np.hypot(tcp[0], tcp[1]) < 0.1:
        return None
    z = np.array([z[0], z[1], 0.0]) / np.hypot(z[0], z[1])
    down = np.array([0.0, 0.0, -1.0])
    yaw = np.degrees(np.arctan2(z[1], z[0]) - np.arctan2(tcp[1], tcp[0]))
    roll = np.degrees(np.arctan2(R[:, 0] @ np.cross(z, down), R[:, 0] @ down))
    return (yaw + 180) % 360 - 180, roll


def flip_joints(q):
    """Same joints with wrist 3 turned by 180 deg, towards zero (joint range +-360 deg).
    Same as system_hardware.flip_joints."""
    q = list(q)
    q[5] = q[5] - np.pi if q[5] > 0 else q[5] + np.pi
    return q


def tilt_deg(rotation):
    """Angle between the tool z axis of an axis-angle rotation and straight down."""
    R = cv2.Rodrigues(np.asarray(rotation, dtype=float))[0]
    return np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1)))


def rotation_angle(a, b):
    """Angle (rad) between two axis-angle rotations."""
    R = cv2.Rodrigues(np.asarray(a, dtype=float))[0].T @ cv2.Rodrigues(np.asarray(b, dtype=float))[0]
    return np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))


def segment_hits_box(p0, p1, lo, hi, reach):
    """True when the segment p0 -> p1 passes through the box lo..hi grown by reach on
    every side (slab test)."""
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    lo, hi = np.asarray(lo, dtype=float) - reach, np.asarray(hi, dtype=float) + reach
    d = p1 - p0
    t0, t1 = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if p0[i] < lo[i] or p0[i] > hi[i]:
                return False
            continue
        a, b = sorted(((lo[i] - p0[i]) / d[i], (hi[i] - p0[i]) / d[i]))
        t0, t1 = max(t0, a), min(t1, b)
        if t0 > t1:
            return False
    return True


def zone_hit(points, reach):
    """Index of the first EXCLUSION_ZONES box the path through points (base x, y, z)
    enters, the box grown by reach; None if none. A zone the path starts in is
    ignored: leaving one is always allowed, so the arm can never get stuck in it."""
    pts = [np.asarray(p, dtype=float)[:3] for p in points]
    pts = pts * 2 if len(pts) == 1 else pts
    for i, (lo, hi) in enumerate(EXCLUSION_ZONES):
        if segment_hits_box(pts[0], pts[0], lo, hi, reach):
            continue
        if any(segment_hits_box(a, b, lo, hi, reach) for a, b in zip(pts, pts[1:])):
            return i
    return None


def joint_path_zone_hit(c, waypoints, reach):
    """zone_hit() for the TCP of moveJs through waypoints (system_arm model, nominal DH)."""
    if not EXCLUSION_ZONES:
        return None
    T_tool = system_arm.pose_matrix(c.getTCPOffset())
    return zone_hit([system_arm.tcp_matrix(list(q), T_tool)[:3, 3] for q in joint_samples(waypoints)], reach)


def draw_zones(image, finder):
    """The exclusion boxes, red, as the overhead camera sees them."""
    T_cam_base = np.linalg.inv(finder.T_base_cam)
    rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
    for i, (lo, hi) in enumerate(EXCLUSION_ZONES):
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        if np.any((T_cam_base[:3, :3] @ corners.T + T_cam_base[:3, 3:4])[2] <= 0):
            continue                                          # behind the camera
        pixels, _ = cv2.projectPoints(corners, rvec, T_cam_base[:3, 3], finder.K, None)
        p = [tuple(int(v) for v in px) for px in pixels.reshape(-1, 2)]
        for a in range(8):
            for b in range(a + 1, 8):
                if bin(a ^ b).count("1") == 1:              # corners differing in one axis: an edge
                    cv2.line(image, p[a], p[b], (0, 0, 255), 1)
        cv2.putText(image, f"zone {i}", p[7], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)


def self_gap(q, T_flange_tcp):
    """Smallest gap (m) between the gripper / wrist 3 and the upper arm / forearm, in the
    system_arm capsule model. Its radii are conservative: the taught spray pose (tool
    pointing back at the elbow end of the forearm, wrist 2 at 179 deg) is -20 mm there
    although the arm reaches it, so this is compared with the taught poses, not with 0."""
    caps = {c[0]: c for c in system_arm.link_capsules(q, T_flange_tcp)}
    s = np.linspace(0, 1, 12)[:, None]
    gap = np.inf
    for moving in ("gripper", "wrist 3"):
        for fixed in ("upper arm", "forearm"):
            _, a0, a1, ra, _ = caps[moving]
            _, b0, b1, rb, _ = caps[fixed]
            pa, pb = a0 + s * (a1 - a0), b0 + s * (b1 - b0)
            distance = np.min(np.linalg.norm(pa[:, None, :] - pb[None, :, :], axis=2))
            gap = min(gap, distance - ra - rb)
    return gap


def joint_samples(waypoints):
    """Joint vectors along moveJs through waypoints, every SELF_CHECK_STEP_DEG."""
    for a, b in zip(waypoints, waypoints[1:]):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        count = max(1, int(np.ceil(np.max(np.abs(b - a)) / np.radians(SELF_CHECK_STEP_DEG))))
        for k in range(count + 1):
            yield a + (b - a) * k / count


def unwind_waypoints(q, gap_of, z_of, min_gap):
    """moveJ waypoints turning every wrist wound past UNWIND_ABOVE_DEG back a full turn,
    one joint at a time, the same pose at the end; None when no plan keeps the gripper
    min_gap from the arm (gap_of(q): self_gap; z_of(q): TCP height, for the swing).

    Turning two wrists at once swings the gripper through the forearm. Wrist 2 first:
    a full wrist-2 turn has to pass the tool pointing back at the arm, and how close
    that gets depends on wrist 1 (-109 mm in the model with wrist 1 at 80 deg, -20 mm,
    as tight as the taught spray pose, with it 105 deg further), so wrist 1 is first
    moved to the nearest angle (UNWIND_PARK_STEP_DEG steps) where that passes. Then
    wrist 2 is parked with the tool pointing out along the wrist-1 axis while wrist 1
    goes to its target (back from parking and / or a full turn: the tool stays clear),
    wrist 3 (only spins the tool about its own axis), wrist 2 to its target."""
    q = np.asarray(q, dtype=float)
    target = q.copy()
    for i in (3, 4, 5):
        if abs(q[i]) > np.radians(UNWIND_ABOVE_DEG):
            target[i] -= 2 * np.pi * np.sign(q[i])

    def build(park_wrist_1):
        path = [q]

        def go(joint, value):
            if abs(path[-1][joint] - value) > 1e-9:
                nxt = path[-1].copy()
                nxt[joint] = value
                path.append(nxt)

        if target[4] != q[4]:
            go(3, park_wrist_1)
            go(4, target[4])
        if abs(path[-1][3] - target[3]) > 1e-9:
            go(4, 2 * np.pi * np.round(path[-1][4] / (2 * np.pi)))     # tool out along the wrist-1 axis
            go(3, target[3])
        go(5, target[5])
        go(4, target[4])
        return path

    limit = np.radians(360 - JOINT_LIMIT_MARGIN_DEG)
    candidates = []
    for step in range(0, 181, UNWIND_PARK_STEP_DEG):
        for sign in ((1,) if step in (0, 180) else (1, -1)):
            park = q[3] + sign * np.radians(step)
            if abs(park) > limit or (target[4] == q[4] and step):
                continue
            path = build(park)
            samples = list(joint_samples(path))
            gap = min(gap_of(x) for x in samples)
            if gap >= min_gap:
                z = [z_of(x) for x in samples]
                candidates.append((step, max(z) - min(z), path))
        if candidates:
            return min(candidates, key=lambda c: c[:2])[2]
    return None


# What each step needs held before it and leaves held after it: None = nothing,
# "top" / "side" = the glass held with that grip, "held" = either grip
STEP_GRIPS = {"pick_top": (None, "top"), "pick_side": (None, "side"), "flip": ("side", "side"),
              "spray": ("held", "held"), "sponge": ("held", "held"),
              "dry": ("held", "held"), "set_down": ("held", None), "place": ("held", None)}
STATION_STEPS = {"spray", "sponge", "dry"}   # hold the glass at a taught pose, chained (to_station)
GRIP_WORDS = {None: "nothing held", "top": "a top grip", "side": "a side grip", "held": "a glass held"}


def check_sequence(sequence):
    """Raise TaskFailed if the steps don't fit together, before anything moves."""
    if not sequence:
        raise TaskFailed("SEQUENCE is empty")
    held = None
    for i, step in enumerate(sequence, 1):
        if step not in STEP_GRIPS:
            raise TaskFailed(f"SEQUENCE step {i} '{step}' is unknown (steps: {', '.join(STEP_GRIPS)})")
        needed, after = STEP_GRIPS[step]
        if (held is None) if needed == "held" else (held != needed):
            raise TaskFailed(f"SEQUENCE step {i} '{step}' needs {GRIP_WORDS[needed]}, "
                             f"there is {GRIP_WORDS[held]}")
        held = held if after == "held" else after
    if held is not None:
        raise TaskFailed("SEQUENCE ends with the glass still held")


class SequenceTask(Task):
    """The SEQUENCE steps on one glass. Each step is a generator method step_<name>;
    check_sequence() makes sure they fit together before anything moves. Every step
    starts and ends with the tool at carry height, and keeps this state up to date:

      glass_xy    base x, y of the glass axis: the camera's at first, then taken from
                  the cup position after every grip and put-down
      held        None, "top" or "side"
      hang        m, TCP above the bottom of the glass while held (or where it was
                  released), so the carry height clears the other glasses
      tip_offset  TCP x, y minus glass_xy with the current grip
      rotation    the tool orientation move_to() uses
    """

    def __init__(self, r, c, suction, glass, place_xy, table_z, glass_height, sequence=SEQUENCE,
                 sprayer=None, sponge=None, on_placed=None):
        self.suction = suction
        self.sprayer = sprayer
        self.sponge = sponge            # bus_servos.BusServo turning the sponge
        self.glass_xy = np.array([glass.x, glass.y])
        self.radius = SIDE_GRIP_RADIUS if SIDE_GRIP_RADIUS is not None else glass.diameter / 2
        self.place_xy = np.asarray(place_xy)
        self.table_z = table_z
        self.glass_height = glass_height
        self.sequence = list(sequence)
        self.held = None
        self.hang = glass_height
        self.tip_offset = np.zeros(2)
        self.flipped = False         # wrist 3 turned by flip(), turned back by the next turn()
        self.station_return_q = None  # joints before the first station (spray, dry), see to_station
        self.station_exit = None      # pose to back out to from the current station
        self.on_placed = on_placed      # called once the glass is down on its tag
        super().__init__(r, c)

    @property
    def carry_z(self):
        # The glass hangs self.hang below the tip, over glasses glass_height tall; never
        # below the traverse plane
        return max(self.table_z + self.glass_height + CARRY_CLEARANCE + self.hang, self.traverse_z)

    @property
    def traverse_z(self):
        return self.table_z + TRAVERSE_HEIGHT

    def turn_floor(self, safe_z):
        """Lowest TCP z allowed on a turn / unwinding arc at safe_z: TURN_MAX_DIP below
        it, but never below the traverse plane (the turn is raised instead)."""
        return max(safe_z - TURN_MAX_DIP, self.traverse_z)

    @property
    def flip_z(self):
        # Turning around the tool z axis (horizontal), the glass sweeps a circle through
        # its farthest point: its far end along the glass axis, one radius to the side
        sweep = np.hypot(max(self.hang, self.glass_height - self.hang), self.radius)
        return max(self.table_z + self.glass_height + CARRY_CLEARANCE + sweep, self.traverse_z)

    # --- motion helpers ---------------------------------------------------------------
    def move_to(self, xyz, label, holding=False, rotation=None, speed=MOVE_SPEED, station=False):
        """moveL; refused before it is sent if it would enter an exclusion zone (not
        for station moves: the cleaning devices are in the zones)."""
        xyz = [xyz[0], xyz[1], min(max(xyz[2], MIN_TCP_Z), MAX_TCP_Z)]
        rotation = self.rotation if rotation is None else rotation
        if not station:
            self.check_zones([self.r.getActualTCPPose()[:3], xyz], label)
        self.move_l(xyz + list(rotation), speed, MOVE_ACCEL)
        steps = self.wait_move(label)
        yield from self.watch_grip(steps, self.c.stopL) if holding else steps

    def zone_reach(self):
        """How far around the TCP must stay out of the zones: the gripper body, and with
        a glass held a sphere around the TCP that holds the whole glass."""
        reach = EXCLUSION_MARGIN
        if self.held is not None:
            reach += np.hypot(self.glass_height, self.radius)
        return reach

    def check_zones(self, points, label):
        """Refuse a move along points (base x, y, z) that would enter an exclusion zone."""
        zone = zone_hit(points, self.zone_reach())
        if zone is not None:
            raise TaskFailed(f"{label} would enter exclusion zone {zone}, not moving")

    def check_zones_joints(self, waypoints, label):
        zone = joint_path_zone_hit(self.c, waypoints, self.zone_reach())
        if zone is not None:
            raise TaskFailed(f"{label} would enter exclusion zone {zone}, not moving")

    def watch_grip(self, steps, stop):
        """Pass a move's steps through; stop it if the vacuum reports the glass lost."""
        for status in steps:
            if self.suction.grip_result() == "LOST":
                stop(STOP_DECEL)
                raise TaskFailed("glass lost while carrying (vacuum still on, r to release)")
            yield status

    def push(self, direction, max_travel, force_limit, label):
        """Slowly along direction until the force sensor feels contact or max_travel is covered."""
        direction = np.asarray(direction, dtype=float)
        direction /= np.linalg.norm(direction)
        start = np.asarray(self.r.getActualTCPPose()[:3])
        self.check_zones([start, start + direction * max_travel], label)
        self.c.zeroFtSensor()
        yield from self.wait(FT_SETTLE, f"{label}: zeroing force sensor")
        start = np.asarray(self.r.getActualTCPPose()[:3])
        while True:
            force = np.linalg.norm(self.r.getActualTCPForce()[:3])
            pos = np.asarray(self.r.getActualTCPPose()[:3])
            if force > force_limit:
                self.c.speedStop(STOP_DECEL)
                print(f"{label}: contact ({force:.1f} N)")
                return
            if (pos - start) @ direction >= max_travel or (direction[2] < 0 and pos[2] <= MIN_TCP_Z):
                self.c.speedStop(STOP_DECEL)
                print(f"{label}: no contact felt, reached max depth")
                return
            self.c.speedL(list(direction * DESCEND_SPEED) + [0, 0, 0], DESCEND_ACCEL, SPEED_CMD_TIME)
            yield f"{label}: force {force:.1f} N"

    def push_down(self, stop_z, force_limit, label):
        """Slowly down until the force sensor feels contact or stop_z is reached."""
        z = self.r.getActualTCPPose()[2]
        yield from self.push([0, 0, -1], z - stop_z, force_limit, label)

    def vacuum_on(self):
        while not self.suction.grip():
            yield "waiting for the release pulse to end"

    def wait_for_grip(self, back_off):
        """Wait for the vacuum; on failure release and move through the back_off points."""
        grip_start = time.time()
        while True:
            result = self.suction.grip_result()
            elapsed = time.time() - grip_start
            if elapsed > GRIP_DWELL and result in ("OK", "UNKNOWN"):
                return
            if elapsed > GRIP_CONFIRM_TIMEOUT:
                self.suction.release()
                for xyz in back_off:
                    yield from self.move_to(xyz, "grip failed, backing off")
                raise TaskFailed(f"grip not confirmed ({result}), released (p to retry)")
            yield f"gripping, waiting for vacuum ({result or 'no report yet'})"

    def gripped(self, grip, tip_offset):
        """After a confirmed grip: the glass axis is where the cup says, not the camera."""
        self.held = grip
        self.tip_offset = np.asarray(tip_offset, dtype=float)
        self.glass_xy = np.asarray(self.r.getActualTCPPose()[:2]) - self.tip_offset

    def turn(self, rotation, label, q_near=None, force=False, keep_turns=False):
        """Turn the tool to rotation at carry height where the tool is (or over the
        start position, TURN_OVER_START). With no turn to make, only up to carry
        height where the tool is.

        The turn is a moveJ to the IK solution nearest q_near (the taught joints for
        that grip, else the joints at the start of the task), not a moveL: going round
        start -> side -> down -> start with moveL lets the controller pick the joint
        path, and a wrist can come back a full turn further each cycle until the
        pendant stops it ("joint close to limits"). This way every turn ends on the
        same joints every cycle, and a flipped wrist 3 is turned back too.
        Between two arm shapes the joints move in a straight line and the TCP along
        an arc that can sag 10 cm or more. The arc is predicted first, and the turn
        happens high enough (up to the ceiling) that it never sags more than
        TURN_MAX_DIP below the safe height."""
        tcp = self.r.getActualTCPPose()
        up = [tcp[0], tcp[1], max(tcp[2], self.carry_z)]
        # No turn only when the orientation AND the arm configuration are right: the
        # same orientation also has a wrist-flipped solution (wrist 1 / 3 +180 deg,
        # wrist 2 mirrored), which ran the gripper into the table beside the glass
        if (not force and not self.flipped
                and rotation_angle(self.rotation, rotation) < np.radians(TURN_TOLERANCE_DEG)
                and (q_near is None or self.configuration_error(q_near) < np.radians(CONFIG_TOLERANCE_DEG))):
            yield from self.move_to(up, "up")
            return
        q_near = self.start_q if q_near is None else q_near
        if TURN_OVER_START:
            x, y = self.start[:2]
            safe_z = max(self.start[2], self.carry_z)
            yield from self.move_to([up[0], up[1], max(up[2], safe_z)], "up")
            yield from self.move_to([x, y, safe_z], "over start position")
        else:
            # Where the tool is: the arc checks keep it above the glasses and clear of the arm
            x, y = up[:2]
            safe_z = self.carry_z
            yield from self.move_to([x, y, max(up[2], safe_z)], "up")
            safe_z = max(safe_z, self.r.getActualTCPPose()[2])

        # Raise the turn until the predicted arc stays up (from the joints the arm
        # will have up there: same orientation, nearest the joints now)
        turn_z = safe_z
        for _ in range(TURN_RAISE_TRIES):
            q_from = None
            if turn_z > safe_z:
                q_from = self.ik([x, y, turn_z] + list(self.rotation), self.r.getActualQ())
            q, lowest = self.turn_plan([x, y, turn_z] + list(rotation), q_near, q_from, keep_turns)
            if lowest >= self.turn_floor(safe_z):
                break
            turn_z += self.turn_floor(safe_z) - lowest + TURN_RAISE_MARGIN
            if turn_z > MAX_TCP_Z:
                raise TaskFailed(f"tool turn sags {(safe_z - lowest) * 1000:.0f} mm, no room to turn "
                                 f"above it under the ceiling (teach SIDE_GRIP_Q / TOP_GRIP_Q closer)")
        else:
            raise TaskFailed(f"tool turn sags to z {lowest:.3f} m even raised to {turn_z:.3f} m")
        if turn_z > safe_z:
            yield from self.move_to([x, y, turn_z], "up to turn height")
            # Checked again from the joints the arm really has now
            q, lowest = self.turn_plan([x, y, turn_z] + list(rotation), q_near, keep_turns=keep_turns)
            if lowest < self.turn_floor(safe_z):
                raise TaskFailed(f"tool turn would sag to z {lowest:.3f} m, more than "
                                 f"{TURN_MAX_DIP * 1000:.0f} mm below the safe height {safe_z:.3f} m")
        self.move_j(q, TURN_SPEED, TURN_ACCEL)
        yield from self.wait_move(label)
        self.flipped = False
        self.rotation = list(rotation)

    def configuration_error(self, q_ref):
        """Largest joint difference (rad) between the joints now and the robot's IK
        solution for the pose now nearest q_ref (by whole turns): ~0 in the arm
        configuration of q_ref, ~180 deg on another branch (wrist flipped...)."""
        q = np.asarray(self.r.getActualQ(), dtype=float)
        ref = np.asarray(q_ref, dtype=float)
        near = ref + 2 * np.pi * np.round((q - ref) / (2 * np.pi))
        solution = self.ik(self.r.getActualTCPPose(), near)
        return float(np.max(np.abs(np.asarray(solution) - q)))

    def check_configuration(self, q_ref, name):
        """Refuse to go on (down to the glass) in another arm configuration than taught."""
        if q_ref is None:
            return
        error = self.configuration_error(q_ref)
        if error > np.radians(CONFIG_TOLERANCE_DEG):
            raise TaskFailed(f"arm is not in the taught {name} configuration "
                             f"({np.degrees(error):.0f} deg off), not going down")

    def ik(self, target, q_near):
        """Joints reaching target nearest q_near, checked with forward kinematics."""
        # Every argument explicit: ur_rtde reads missing ones from stale registers
        # (see safe_motion.getForwardKinematics)
        q = list(self.c.getInverseKinematics(list(target), list(q_near), IK_MAX_ERROR, IK_MAX_ERROR))
        reached = self.c.getForwardKinematics(q, self.c.getTCPOffset())
        if (np.linalg.norm(np.subtract(reached[:3], target[:3])) > TURN_IK_TOLERANCE
                or rotation_angle(reached[3:], target[3:]) > np.radians(TURN_TOLERANCE_DEG)):
            raise TaskFailed("no joint solution for the tool turn")
        return q

    def turn_plan(self, target, q_near, q_from=None, keep_turns=False, zones=True):
        """(joints for target nearest q_near, lowest TCP z on the joint arc from q_from).

        q_from defaults to the joints now. Unless keep_turns, q_near is shifted by
        whole turns to the ones the joints have: the same arm shape without turning
        a wrist a full circle (that swings the TCP ~10 cm down on the arc). Where
        that would end near a joint limit, the unwound equivalent is used instead:
        the wrist then turns back a full circle on this move (turn() raises the
        turn for the swing). unwind() at the start of the task makes that rare."""
        q0 = np.asarray(self.r.getActualQ() if q_from is None else q_from, dtype=float)
        q_near = np.asarray(q_near, dtype=float)
        if not keep_turns:
            shifted = q_near + 2 * np.pi * np.round((q0 - q_near) / (2 * np.pi))
            # Only where that saves more than half a turn (a wound wrist); a change of
            # configuration (~180 deg) takes the plain, unwound angles
            q_near = np.where(np.abs(q0 - q_near) - np.abs(q0 - shifted) > np.pi, shifted, q_near)
            limit = np.radians(360 - JOINT_LIMIT_MARGIN_DEG)
            q_near = np.where(np.abs(q_near) > limit, q_near - 2 * np.pi * np.sign(q_near), q_near)
        q = self.ik(target, q_near)
        near = joint_near_limit(q)
        if near is not None:
            raise TaskFailed(f"tool turn would take {near[0]} to {near[1]:.0f} deg, near its limit")
        return q, self.arc_lowest(q0, q, "tool turn", zones)

    def arc_lowest(self, q0, q1, label, zones=True):
        """Lowest TCP z on a moveJ from q0 to q1 (joints in a line, the TCP on an arc).
        Refused when the gripper would come closer to the arm than any taught pose."""
        q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
        self.check_self_gap([q0, q1], label)
        if zones:
            self.check_zones_joints([q0, q1], label)
        offset = self.c.getTCPOffset()
        z = [self.c.getForwardKinematics(list(q0 + t * (q1 - q0)), offset)[2]
             for t in np.linspace(0, 1, TURN_CHECK_STEPS + 1)[1:]]
        if max(z) > MAX_TCP_Z:
            raise TaskFailed(f"{label} would rise to z {max(z):.3f} m, above the ceiling")
        return min(z)

    def put_down(self, glass_xy, label):
        """Lower the held glass onto the table with its axis at glass_xy, release, leave."""
        tx, ty = np.asarray(glass_xy) + self.tip_offset
        z = self.table_z + self.hang                     # TCP with the glass standing on the table
        yield from self.move_to([tx, ty, self.carry_z], f"carry to {label}", holding=True)
        yield from self.move_to([tx, ty, z + APPROACH_GAP], "lower", holding=True, speed=APPROACH_SPEED)
        yield from self.push_down(z - MAX_OVERSHOOT, PLACE_FORCE, label)
        self.suction.release()
        yield from self.wait(RELEASE_TIME, "releasing")
        tcp = np.asarray(self.r.getActualTCPPose()[:3])
        self.glass_xy = tcp[:2] - self.tip_offset
        if self.held == "side":
            # Back off sideways before going up so the cup does not drag the glass
            tcp = tcp - self.approach * SIDE_STANDOFF
            yield from self.move_to(list(tcp), "back off")
        self.held = None
        yield from self.move_to([tcp[0], tcp[1], self.carry_z], "up")

    # --- the steps (SEQUENCE names) ---------------------------------------------------
    def step_pick_top(self):
        """Grab the top of the standing glass, tool pointing down."""
        self.hang = self.glass_height
        x, y = self.glass_xy + np.asarray(TOP_GRIP_OFFSET)
        top_z = self.table_z + self.glass_height         # cup lands on top of the glass
        yield from self.turn(self.down_rotation, "turn tool down", TOP_GRIP_Q)
        yield from self.move_to([x, y, self.carry_z], "to glass", speed=APPROACH_SPEED)
        self.check_configuration(TOP_GRIP_Q, "top grip")
        yield from self.move_to([x, y, top_z + APPROACH_GAP], "approach glass", speed=APPROACH_SPEED)
        yield from self.vacuum_on()
        yield from self.push_down(top_z - TOP_PICK_OVERSHOOT, CONTACT_FORCE, "pick")
        yield from self.wait_for_grip([[x, y, self.carry_z]])
        self.gripped("top", TOP_GRIP_OFFSET)
        tcp = self.r.getActualTCPPose()
        yield from self.move_to([tcp[0], tcp[1], self.carry_z], "lift", holding=True)

    def step_pick_side(self):
        """Grab the glass wall at SIDE_GRIP_HEIGHT, tool horizontal."""
        glass_xy = self.glass_xy
        if np.linalg.norm(glass_xy) < 0.1:
            raise TaskFailed("glass is too close to the robot base for a side grip")
        if SIDE_GRIP_ROTATION is not None:
            # Taught orientation as it is, approach along its tool z axis (may dip a little)
            rotation = list(SIDE_GRIP_ROTATION)
            approach = cv2.Rodrigues(np.asarray(SIDE_GRIP_ROTATION, dtype=float))[0][:, 2]
            d = approach[:2] / np.linalg.norm(approach[:2])
        else:
            # Radial approach: tilting the tool outwards is a wrist 1 move, far from the
            # wrist singularity (wrist 2 stays near -90 deg as in HOME_Q)
            yaw = np.radians(SIDE_APPROACH_YAW_DEG)
            radial = glass_xy / np.linalg.norm(glass_xy)
            d = np.array([radial[0] * np.cos(yaw) - radial[1] * np.sin(yaw),
                          radial[0] * np.sin(yaw) + radial[1] * np.cos(yaw)])
            approach = np.array([d[0], d[1], 0.0])
            rotation = side_rotation(d, SIDE_ROLL_DEG)
        self.approach = approach
        self.hang = SIDE_GRIP_HEIGHT
        # SIDE_STANDOFF back along the approach from the wall point at grip height
        wall = np.array([*(glass_xy - d * self.radius), self.table_z + SIDE_GRIP_HEIGHT])
        standoff = wall - approach * SIDE_STANDOFF

        yield from self.turn(rotation, "turn tool sideways", SIDE_GRIP_Q)
        yield from self.move_to([standoff[0], standoff[1], self.carry_z], "to glass", speed=APPROACH_SPEED)
        self.check_configuration(SIDE_GRIP_Q, "side grip")
        yield from self.move_to(list(standoff), "down beside glass", speed=APPROACH_SPEED)
        yield from self.vacuum_on()
        yield from self.push(approach, SIDE_STANDOFF + SIDE_MAX_PRESS, SIDE_CONTACT_FORCE, "pick")
        yield from self.wait_for_grip([list(standoff), [standoff[0], standoff[1], self.carry_z]])
        self.gripped("side", -d * self.radius)
        tcp = self.r.getActualTCPPose()
        yield from self.move_to([tcp[0], tcp[1], self.carry_z], "lift", holding=True)

    def step_flip(self):
        """Turn the side-gripped glass upside down with wrist 3 alone. The glass axis
        crosses the tool z axis, so the glass stays over the spot it was picked from
        (free space) and only its ends swap: the TCP ends up glass_height - hang above
        its new bottom."""
        tcp = self.r.getActualTCPPose()
        yield from self.move_to([tcp[0], tcp[1], self.flip_z], "up to flip height", holding=True)
        q = self.r.getActualQ()
        self.flipped = True
        self.move_j(flip_joints(q), FLIP_SPEED, FLIP_ACCEL)
        yield from self.watch_grip(self.wait_move("turning glass"), self.c.stopJ)
        self.rotation = list(self.r.getActualTCPPose()[3:])
        self.hang = self.glass_height - self.hang
        yield from self.move_to([tcp[0], tcp[1], self.carry_z], "down to carry height", holding=True)

    # --- stations: taught poses the held glass is taken to (spray, dry) -------------
    def station_floor(self, z0, z1):
        """Lowest TCP z allowed on a joint move with the glass held between heights z0 and
        z1: carry height (the glass over the others), or lower when a taught station
        is, minus TURN_MAX_DIP (the arc may sag a little, not below both ends)."""
        return min(self.carry_z, z0, z1) - TURN_MAX_DIP

    def station_plan(self, entry, q_ref, name, keep_turns=False):
        """Joints for a station entry from where the arm is: the TCP arc must keep the
        glass over the others and the gripper clear of the arm (TaskFailed if not).
        keep_turns: arrive with q_ref's own angles, not shifted by whole turns."""
        q_entry, lowest = self.turn_plan(entry, q_ref, keep_turns=keep_turns, zones=False)  # devices in the zones
        if lowest < self.station_floor(self.r.getActualTCPPose()[2], entry[2]):
            raise TaskFailed(f"move to the {name} would sag to z {lowest:.3f} m "
                             f"(lengthen its approach or teach its joints closer)")
        return q_entry

    def to_station(self, pose, q_ref, approach, name, keep_turns=False):
        """Take the held glass to a taught station pose: joint move (in the taught arm
        configuration q_ref, arc checked) to pose + approach, then straight in.
        From another station straight there when the checks pass, otherwise back
        through the joints the arm had before the first station (the direct sprayer ->
        dryer move swung the gripper 94 mm into the arm in the model); leave_stations()
        also takes it back there before any other step."""
        entry = list(np.add(pose[:3], approach)) + list(pose[3:])
        q_entry = None
        if self.station_return_q is not None:
            try:
                q_entry = self.station_plan(entry, q_ref, name, keep_turns)
            except TaskFailed as e:
                print(f"{e}: going back through the joints after the pick instead")
                return_q = self.station_return_q
                yield from self.leave_stations()
                self.station_return_q = return_q
        else:
            self.station_return_q = self.r.getActualQ()
        if q_entry is None:
            q_entry = self.station_plan(entry, q_ref, name, keep_turns)
        self.move_j(q_entry, SPRAY_MOVE_SPEED, SPRAY_MOVE_ACCEL)
        yield from self.watch_grip(self.wait_move(f"to the {name}"), self.c.stopJ)
        yield from self.move_to(pose[:3], f"into the {name} pose", holding=True, station=True,
                                rotation=pose[3:], speed=APPROACH_SPEED)
        self.station_exit = entry

    def out_of_station(self, name):
        """Back out the way it came in, then straight up to the traverse plane before
        anything else (from the low sprayer the next joint move swept along at 146 mm)."""
        # In the orientation the tool has now: the drying turns are about the tool z
        # axis, so the way out is the same line, and a straight move back to the entry
        # orientation would have to undo up to 180 deg (no defined direction)
        rotation = list(self.r.getActualTCPPose()[3:])
        exit_pose = list(self.station_exit[:3]) + rotation
        yield from self.move_to(exit_pose[:3], f"out of the {name} pose", holding=True,
                                rotation=rotation, station=True)
        if exit_pose[2] < self.traverse_z:
            yield from self.move_to([exit_pose[0], exit_pose[1], self.traverse_z], f"up from the {name}",
                                    holding=True, rotation=exit_pose[3:], station=True)
            self.station_exit = [exit_pose[0], exit_pose[1], self.traverse_z] + list(exit_pose[3:])

    def leave_stations(self):
        """Back to the joints before the first station, where the glass hangs as picked
        (the same joints, so nothing winds up)."""
        if self.station_return_q is None:
            return
        back_q = self.station_return_q
        lowest = self.arc_lowest(self.r.getActualQ(), back_q, "move back from the stations", zones=False)
        back_z = self.c.getForwardKinematics(list(back_q), self.c.getTCPOffset())[2]
        if lowest < self.station_floor(self.r.getActualTCPPose()[2], back_z):
            raise TaskFailed(f"move back from the stations would sag to z {lowest:.3f} m")
        self.move_j(list(back_q), SPRAY_MOVE_SPEED, SPRAY_MOVE_ACCEL)
        yield from self.watch_grip(self.wait_move("back from the stations"), self.c.stopJ)
        self.station_return_q = None

    def step_spray(self):
        """Hold the glass in the taught SPRAY_POSE and work the sprayer."""
        yield from self.to_station(SPRAY_POSE, SPRAY_Q, SPRAY_APPROACH, "sprayer")
        # A burst from the y / Back button still going: Sprayer.start() would block
        # the loop until it ends (watchdog), so let it end here
        if self.sprayer.running:
            self.sprayer.request_stop()
            while self.sprayer.running:
                yield "waiting for the manual spray to end"
        self.sprayer.start(period=SPRAY_PERIOD, count=SPRAY_STROKES)
        try:
            end = time.time() + SPRAY_TIMEOUT
            while self.sprayer.running:
                if time.time() > end:
                    raise TaskFailed(f"sprayer not done after {SPRAY_TIMEOUT:.0f} s (glass still held)")
                if self.suction.grip_result() == "LOST":
                    raise TaskFailed("glass lost while spraying (vacuum still on, r to release)")
                yield f"spraying, stroke {self.sprayer.strokes + 1}/{SPRAY_STROKES}"
            if self.sprayer.error is not None:
                raise TaskFailed(f"sprayer failed ({self.sprayer.error}), glass still held")
        finally:
            # Also on s / abort: stop after the current stroke without blocking the loop
            self.sprayer.request_stop()
        yield from self.out_of_station("sprayer")

    def step_sponge(self):
        """Hold the glass on the sponge (SPONGE_POSE, from above) and turn the sponge
        SPONGE_SPIN_TIME each way. The servo is stopped on the way out, also on s /
        abort / failure; the glass stays held."""
        yield from self.to_station(SPONGE_POSE, SPONGE_Q, SPONGE_APPROACH, "sponge")
        servo_errors = (TimeoutError, bus_servos.ServoError, SerialException)
        try:
            for direction, name in ((1, "one way"), (-1, "the other way")):
                self.sponge.spin(direction * SPONGE_SPEED)
                end = time.time() + SPONGE_SPIN_TIME
                while time.time() < end:
                    if self.suction.grip_result() == "LOST":
                        raise TaskFailed("glass lost on the sponge (vacuum still on, r to release)")
                    yield f"sponge turning {name}, {end - time.time():.0f} s"
        except servo_errors as e:
            raise TaskFailed(f"sponge servo not answering ({e}), glass still held")
        finally:
            try:
                self.sponge.stop()
            except servo_errors:
                pass
        yield from self.out_of_station("sponge")

    def step_dry(self):
        """Hold the glass in the taught DRY_POSE, coming in along the tool z axis, and
        turn it about that axis (wrist 3: the TCP stays, the glass turns about its own
        axis with the top grip) to each angle of DRY_TURNS_DEG."""
        tool_z = cv2.Rodrigues(np.asarray(DRY_POSE[3:], dtype=float))[0][:, 2]
        pose = list(np.add(DRY_POSE[:3], DRY_DEPTH * tool_z)) + list(DRY_POSE[3:])
        # With DRY_Q's own angles (wrist 3 -143 deg), so the turns always end the same
        # (+180 -> +37 deg); arriving a turn wound, +180 would run into the joint limit
        yield from self.to_station(pose, DRY_Q, -DRY_APPROACH * tool_z, "drying", keep_turns=True)
        arrived = list(self.r.getActualQ())
        targets = [arrived[:5] + [arrived[5] + np.radians(angle)] for angle in DRY_TURNS_DEG]
        for q in targets:
            near = joint_near_limit(q)
            if near is not None:
                raise TaskFailed(f"drying turn would take {near[0]} to {near[1]:.0f} deg, near its limit")
        for i, q in enumerate(targets, 1):
            self.move_j(q, DRY_SPEED, DRY_ACCEL)
            yield from self.watch_grip(self.wait_move(f"drying, turn {i}/{len(targets)}"), self.c.stopJ)
        yield from self.out_of_station("drying")

    def step_set_down(self):
        """Put the held glass down where it is: the spot it was picked from is free."""
        yield from self.put_down(self.glass_xy, "set down")

    def step_place(self):
        """Put the held glass down on the place tag."""
        yield from self.put_down(self.place_xy, "tag")
        # The tag is taken from here on, even if the way back fails or is aborted
        if self.on_placed is not None:
            self.on_placed()

    # --- the whole task -----------------------------------------------------------------
    def check_start(self):
        """Refuse what can't work before anything moves."""
        check_sequence(self.sequence)
        # Joint moves may come as close to the arm as the closest pose taught by hand
        T_tool = system_arm.pose_matrix(self.c.getTCPOffset())
        taught = [self.start_q] + [q for q in (SIDE_GRIP_Q, TOP_GRIP_Q, SPRAY_Q, DRY_Q, START_Q) if q is not None]
        self.min_self_gap = min(SELF_GAP_PROVEN, *(self_gap(q, T_tool) for q in taught)) - SELF_GAP_MARGIN
        if "pick_top" in self.sequence:
            # Taught, else the start orientation if it points down (as before), else home
            candidates = ([TOP_GRIP_ROTATION] if TOP_GRIP_ROTATION is not None else
                          [self.start[3:], self.c.getForwardKinematics(START_Q)[3:]])
            for rotation in candidates:
                if tilt_deg(rotation) <= MAX_TILT_DEG:
                    self.down_rotation = list(rotation)
                    break
            else:
                source = "TOP_GRIP_ROTATION" if TOP_GRIP_ROTATION is not None else "START_Q"
                raise TaskFailed(f"{source} does not point the tool down, can't pick from the top")
        if "sponge" in self.sequence and self.sponge is None:
            raise TaskFailed(f"SEQUENCE sponges, but the sponge servo on {SPRAYER_PORT} is not connected")
        if "spray" in self.sequence and self.sprayer is None:
            raise TaskFailed(f"SEQUENCE sprays, but the sprayer on {SPRAYER_PORT} is not connected")
        if "flip" in self.sequence or "dry" in self.sequence:
            offset = self.c.getTCPOffset()
            if np.hypot(offset[0], offset[1]) > MAX_TCP_XY_OFFSET:
                raise TaskFailed("TCP offset has x/y (pendant), wrist 3 turns (flip, dry) would swing the glass")
        # Highest carry height of each step, the glass either way up
        side_hang = max(SIDE_GRIP_HEIGHT, self.glass_height - SIDE_GRIP_HEIGHT)
        extra = {"pick_top": self.glass_height, "pick_side": side_hang,
                 "flip": np.hypot(side_hang, self.radius)}
        for step in set(self.sequence) & extra.keys():
            z = self.table_z + self.glass_height + CARRY_CLEARANCE + extra[step]
            if z > MAX_TCP_Z:
                raise TaskFailed(f"{step}: carry height {z:.3f} m is above MAX_TCP_Z")

    def check_self_gap(self, waypoints, label):
        """Refuse moveJs through waypoints that bring the gripper too close to the arm
        (before the pendant does, with "tool hitting the arm" and a protective stop)."""
        T_tool = system_arm.pose_matrix(self.c.getTCPOffset())
        gap = min(self_gap(q, T_tool) for q in joint_samples(waypoints))
        if gap < self.min_self_gap:
            raise TaskFailed(f"{label} would bring the gripper {(self.min_self_gap - gap) * 1000:.0f} mm "
                             f"closer to the arm than any taught pose, not moving")

    def unwind(self):
        """Turn wound-up wrists back a full turn at the start of the task, gripper
        empty, one joint at a time (unwind_waypoints): the same pose with the joint
        angles back near zero, so no turn later in the task runs into the +-360 deg
        joint limits ("joint close to limits" on the pendant). Jogging and moveL wind
        them up a turn at a time. Done high up where the arm stands, the TCP arc
        predicted with the model and the unwinding raised until it stays up."""
        q = np.asarray(self.start_q, dtype=float)
        if np.any(np.abs(q[:3]) > np.radians(UNWIND_ABOVE_DEG)):
            raise TaskFailed("base joint wound up a full turn, turn it back on the pendant first")
        if not np.any(np.abs(q[3:]) > np.radians(UNWIND_ABOVE_DEG)):
            return
        T_tool = system_arm.pose_matrix(self.c.getTCPOffset())
        safe_z = max(self.start[2], self.carry_z)

        def plan():
            waypoints = unwind_waypoints(self.r.getActualQ(), lambda x: self_gap(x, T_tool),
                                         lambda x: system_arm.tcp_matrix(list(x), T_tool)[2, 3],
                                         self.min_self_gap)
            if waypoints is None:
                raise TaskFailed("no way to unwind the wrists without the gripper coming closer to "
                                 "the arm than any taught pose; unwind on the pendant")
            z = [system_arm.tcp_matrix(list(p), T_tool)[2, 3] for p in joint_samples(waypoints)]
            # Model heights relative to where the arm really is (nominal DH is a few mm off)
            shift = self.r.getActualTCPPose()[2] - system_arm.tcp_matrix(list(waypoints[0]), T_tool)[2, 3]
            return waypoints, min(z) + shift, max(z) + shift

        tcp = self.r.getActualTCPPose()
        yield from self.move_to([tcp[0], tcp[1], max(tcp[2], safe_z)], "up to unwind")
        waypoints, lowest, highest = plan()
        if lowest < self.turn_floor(safe_z):
            rise = self.turn_floor(safe_z) - lowest + TURN_RAISE_MARGIN
            if highest + rise > MAX_TCP_Z:
                raise TaskFailed(f"unwinding swings the TCP {(highest - lowest) * 1000:.0f} mm up and down, "
                                 f"no room under the ceiling; unwind on the pendant")
            tcp = self.r.getActualTCPPose()
            yield from self.move_to([tcp[0], tcp[1], tcp[2] + rise], "up to unwind")
            waypoints, lowest, highest = plan()
            if lowest < self.turn_floor(safe_z):
                raise TaskFailed(f"unwinding would sag to z {lowest:.3f} m, not moving")
        self.check_self_gap(waypoints, "unwinding")
        self.check_zones_joints(waypoints, "unwinding")
        names = ", ".join(f"{JOINT_NAMES[i]} {np.degrees(waypoints[0][i]):.0f} -> "
                          f"{np.degrees(waypoints[-1][i]):.0f} deg"
                          for i in (3, 4, 5) if abs(waypoints[-1][i] - waypoints[0][i]) > 1)
        print(f"Unwinding: {names}")
        for k, q_next in enumerate(waypoints[1:], 1):
            self.move_j(list(q_next), TURN_SPEED, TURN_ACCEL)
            yield from self.wait_move(f"unwinding {k}/{len(waypoints) - 1} ({names})")
        self.start_q = list(self.r.getActualQ())
        self.rotation = list(self.r.getActualTCPPose()[3:])

    def run(self):
        # Any start orientation: each pick turns the tool if it
        # needs another one, and it turns back to the start orientation at the end
        self.start = self.r.getActualTCPPose()
        self.start_q = self.r.getActualQ()        # every tool turn ends near these joints
        self.rotation = list(self.start[3:])
        self.check_start()
        yield from self.unwind()
        print(f"Glass at {self.glass_xy[0] * 1000:.0f}, {self.glass_xy[1] * 1000:.0f} mm "
              f"(radius {self.radius * 1000:.0f} mm), place at {self.place_xy[0] * 1000:.0f}, "
              f"{self.place_xy[1] * 1000:.0f} mm: {' > '.join(self.sequence)}")

        for i, step in enumerate(self.sequence, 1):
            steps = getattr(self, "step_" + step)()
            if step not in STATION_STEPS:
                steps = chain(self.leave_stations(), steps)
            for status in steps:
                yield f"{i}/{len(self.sequence)} {step}: {status}"

        # Out of the camera's way
        start = self.start
        yield from self.turn(start[3:], "turn tool back")
        yield from self.move_to([start[0], start[1], max(start[2], self.carry_z)], "back")
        yield from self.move_to(start[:3], "back")
        self.status = "done"


def connect_servos():
    """(ServoBus, Sprayer, sponge BusServo) on SPRAYER_PORT; None for what SEQUENCE does not
    use or does not answer (with a warning: p is then refused, y needs the sprayer)."""
    if not {"spray", "sponge"} & set(SEQUENCE):
        return None, None, None
    try:
        bus = bus_servos.ServoBus(SPRAYER_PORT)
    except SerialException as e:
        warn(f"servo board on {SPRAYER_PORT} not available ({e}), p will be refused")
        return None, None, None
    servos = {}
    for step, servo_id, name in (("spray", bus_servos.SPRAYER_ID, "sprayer"),
                                 ("sponge", bus_servos.ROTATOR_ID, "sponge")):
        servo = bus_servos.BusServo(bus, servo_id)
        if servo.ping():
            servos[step] = servo
        elif step in SEQUENCE:
            warn(f"{name} servo {servo_id} on {SPRAYER_PORT} not answering, p will be refused")
    sprayer = bus_servos.Sprayer(servos["spray"]) if "spray" in servos else None
    return bus, sprayer, servos.get("sponge")


def spray_command(sprayer, task):
    """y / Back: a burst of MANUAL_SPRAY_STROKES, or stop the one running. Refused while
    a task runs (it works the sprayer itself) or without the sprayer. Never blocks."""
    if sprayer is None:
        return warn(f"no sprayer on {SPRAYER_PORT}, y ignored")
    if task is not None:
        return warn("task running, y ignored (s stops the task)")
    if sprayer.running:
        sprayer.request_stop()
        return "spray stopping after this stroke"
    if sprayer.error is not None:
        warn(f"last spray failed ({sprayer.error}), trying again")
    sprayer.start(period=SPRAY_PERIOD, count=MANUAL_SPRAY_STROKES)
    return f"spraying {MANUAL_SPRAY_STROKES} strokes (y / Back again stops)"


def find_place_tags(detector, finder, image):
    """{id: base x, y of its center on the table} for the place tags in view."""
    tags = {}
    for tag_id, pixel in fg.tag_centers(detector, image).items():
        if PLACE_TAG_IDS is not None and tag_id not in PLACE_TAG_IDS:
            continue
        p = fg.pixel_to_plane(finder.K, finder.T_base_cam, pixel, finder.table_z)
        if p is not None:
            tags[tag_id] = np.asarray(p[:2])
    return tags


def on_tag(glass, xy):
    return np.hypot(glass.x - xy[0], glass.y - xy[1]) <= PLACED_RADIUS


def next_place_tag(tags, used, glasses):
    """Id of the tag to put the next glass on: the first (by id, or PLACE_TAG_IDS order)
    not used this round and with no glass standing on it; None when all are taken."""
    order = PLACE_TAG_IDS if PLACE_TAG_IDS is not None else sorted(tags)
    for tag_id in order:
        if tag_id in tags and tag_id not in used and not any(on_tag(g, tags[tag_id]) for g in glasses):
            return tag_id
    return None


def choose_glass(glasses, tags):
    """The glass closest to the robot base (horizontal distance from its axis), skipping
    glasses standing on a place tag (done already)."""
    free = [g for g in glasses if not any(on_tag(g, xy) for xy in tags.values())]
    if len(free) < len(glasses):
        print(f"(skipping {len(glasses) - len(free)} glass(es) standing on a place tag)")
    return min(free, key=lambda g: np.hypot(g.x, g.y), default=None)


def draw_places(image, finder, tags, seen, used, next_id):
    """Every known place tag: the next one bright cyan, used ones gray, others dark cyan."""
    T_cam_base = np.linalg.inv(finder.T_base_cam)
    rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
    for tag_id, xy in tags.items():
        point = np.array([[xy[0], xy[1], finder.table_z]])
        pixel, _ = cv2.projectPoints(point, rvec, T_cam_base[:3, 3], finder.K, None)
        p = tuple(int(v) for v in pixel.ravel())
        if tag_id == next_id:
            color, label = (255, 255, 0), f"tag {tag_id}: next"
        elif tag_id in used:
            color, label = (140, 140, 140), f"tag {tag_id}: used"
        else:
            color, label = (160, 160, 0), f"tag {tag_id}"
        if tag_id not in seen:
            label += " (last seen)"
        cv2.drawMarker(image, p, color, cv2.MARKER_SQUARE, 30, 2)
        cv2.putText(image, label, (p[0] + 18, p[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main():
    check_sequence(SEQUENCE)     # a SEQUENCE typo fails here, before anything connects
    cap, width, height = fg.open_camera()
    grabber = FrameGrabber(cap)
    finder = fg.GlassFinder.load(width, height, rim_height=GLASS_HEIGHT)
    detector = fg.make_tag_detector()
    editor = fg.AreaEditor(finder)
    fg.setup_window(editor)

    suction = connect_suction()
    servo_bus, sprayer, sponge = connect_servos()
    r, c = safe_motion.connect(IP)
    gamepad = GamepadControl(GAMEPAD_KEYS)
    jogger = Jogger(c, gamepad, MIN_TCP_Z, MAX_TCP_Z)
    print("Connected to robot. TCP pose:", r.getActualTCPPose())

    task = None
    tags = {}                        # place tag id -> last seen base x, y
    seen_tags = {}
    used_tags = set()                # glass put on it this round (c clears)
    batch = False                    # PICK_ALL run going: the next glass after this one
    batch_used = 0                   # len(used_tags) when p was pressed
    status = "p: pick & place  s: stop  g/r: grip/release  h: home  f: freedrive  q: quit"
    freedrive = False
    zone_corner = None               # first corner taught with b
    last_frame = None
    image = None
    glasses = []
    watchdog = RobotWatchdog(c, r)
    try:
        while True:
            if fg.window_closed():
                break
            loop_start = time.time()
            try:
                frame_count, frame = grabber.latest()
                new_frame = frame_count != last_frame
                if new_frame:
                    last_frame = frame_count
                    fg.read_trackbars(finder)
                    image = finder.undistort(frame)
                if not watchdog.kick():
                    if task is not None:
                        task.abort()
                        task = None
                    jogger.stop()
                    freedrive = False       # the re-uploaded control script is not in teach mode
                    status = "robot was stopped (loop stall / protective stop), task aborted"
                if new_frame and task is None:
                    # Only between tasks (~40 ms): during one the glass is chosen and the
                    # tag position kept, and the robot steps run at LOOP_PERIOD instead
                    glasses = finder.detect(image)
                    seen_tags = find_place_tags(detector, finder, image)
                    tags.update(seen_tags)
                tcp = r.getActualTCPPose()

                pick_now = False             # p pressed at the start, or the start reached after p
                if task is not None:
                    task.update()
                    status = task.status
                    if task.done:
                        pick_now = isinstance(task, HomeTask) and task.then_pick and task.arrived
                        if isinstance(task, SequenceTask) and batch:
                            # PICK_ALL: the next glass as soon as this one is done; a failed
                            # one ends the run (its message stays in the window)
                            pick_now = task.status == "done"
                            batch = pick_now
                        task = None
                # Backstop for moves that got above the ceiling anyway (safe_motion.py)
                if task is not None and c.over_ceiling():
                    task.abort()
                    task = None
                    status = warn(f"above the {MAX_TCP_Z:.2f} m ceiling, task stopped (jog down)")

                if new_frame:
                    next_id = next_place_tag(tags, used_tags, glasses)
                    fg.draw_overlay(image, finder, editor, glasses, status, tcp[:2],
                                    tags[next_id] if next_id is not None else None)
                    draw_places(image, finder, tags, seen_tags if task is None else {}, used_tags, next_id)
                    draw_zones(image, finder)
                    cv2.imshow(fg.WINDOW, image)

                if gamepad.jog_speed() is not None and task is not None:
                    task.abort()
                    task = None
                    status = "manual override, task aborted"
                    print(status)
                if task is None and not freedrive:
                    jogger.update(tcp)

                keys = fg.read_keys(gamepad)
                if "q" in keys or "\x1b" in keys:
                    break
                for key in keys:
                    if key == "s":
                        batch = False                  # ends a PICK_ALL run too
                        if task is not None:
                            task.abort()
                            task = None
                        if freedrive:
                            c.endTeachMode()
                            freedrive = False
                        jogger.stop()
                        c.speedStop(STOP_DECEL)
                        status = "stopped"
                    elif key == "f":
                        if task is not None:
                            status = warn("task running, f ignored (s stops the task)")
                        elif freedrive:
                            c.endTeachMode()
                            freedrive = False
                            status = "freedrive off"
                        else:
                            jogger.stop()
                            c.teachMode()
                            freedrive = True
                            status = "FREEDRIVE: move the arm by hand (f / right stick click ends it)"
                        print(status)
                    elif key in ("h", "p") and freedrive:
                        status = warn(f"freedrive on, {key} ignored (f / right stick click ends it)")
                    elif key == "g" and task is not None:
                        # The pick task controls the vacuum and waits for its GRIP result
                        status = warn("task running, g ignored (s stops the task)")
                    elif key in ("g", "r"):
                        status = key_command(suction, key)
                    elif key == "c":
                        used_tags.clear()
                        status = "place tags cleared, the next p starts with the first tag again"
                        print(status)
                    elif key == "b":
                        # Two opposite corners of an exclusion box, at the TCP
                        tip = [round(v, 3) for v in r.getActualTCPPose()[:3]]
                        if zone_corner is None:
                            zone_corner = tip
                            status = f"zone corner 1 at {tip}: move the tip to the opposite corner, b again"
                        else:
                            lo = [min(a, b) for a, b in zip(zone_corner, tip)]
                            hi = [max(a, b) for a, b in zip(zone_corner, tip)]
                            print(f"EXCLUSION_ZONES = [({lo}, {hi})]   # add to the list, "
                                  f"{(hi[0] - lo[0]) * 1000:.0f} x {(hi[1] - lo[1]) * 1000:.0f} x "
                                  f"{(hi[2] - lo[2]) * 1000:.0f} mm")
                            zone_corner = None
                            status = "zone printed in the console, paste it into EXCLUSION_ZONES"
                        print(status)
                    elif key == "y":
                        status = spray_command(sprayer, task)
                    elif key == "o":
                        pose = r.getActualTCPPose()
                        rotation = ", ".join(f"{v:.5f}" for v in pose[3:])
                        # Joints wrapped to +-180 deg: the same arm pose, never wound up
                        joints = ", ".join(f"{(v + np.pi) % (2 * np.pi) - np.pi:.5f}" for v in r.getActualQ())
                        if tilt_deg(pose[3:]) <= MAX_TILT_DEG:
                            status = f"TOP_GRIP_ROTATION = [{rotation}]"
                            print(f"{status}\nTOP_GRIP_Q = [{joints}]")
                            continue
                        print(f"SIDE_GRIP_ROTATION = [{rotation}]\n"
                              f"SIDE_GRIP_Q = [{joints}]\n"
                              f"SIDE_GRIP_HEIGHT = {pose[2] - finder.table_z:.4f}\n"
                              f"or, holding a glass in front of the sprayer:\n"
                              f"SPRAY_POSE = [{', '.join(f'{v:.5f}' for v in pose)}]\n"
                              f"SPRAY_Q = [{joints}]")
                        side = read_side_orientation(pose)
                        if side is None:
                            status = "o: tool not horizontal, only SIDE_GRIP_ROTATION printed"
                        else:
                            status = f"SIDE_APPROACH_YAW_DEG = {side[0]:.0f}  SIDE_ROLL_DEG = {side[1]:.0f}"
                        print(status)
                    elif key in ("h", "p") and task is not None:
                        status = warn(f"task running ({task.status}), {key} ignored (s stops the task)")
                    elif key == "h":
                        jogger.stop()
                        task = HomeTask(r, c)
                    elif key == "p":
                        jogger.stop()
                        batch = PICK_ALL
                        batch_used = len(used_tags)
                        if not GO_TO_START_FIRST or at_start(r):
                            pick_now = True
                        else:
                            # Out of the camera's view first; the pick starts when it arrives
                            task = HomeTask(r, c, then_pick=True)

                if pick_now and task is None:
                    done = len(used_tags) - batch_used       # glasses placed since p
                    if not tags:
                        status = warn("no place tag in view, p ignored")
                        batch = False
                        continue
                    status = "measuring..."
                    measured = finder.measure(grabber, kick=watchdog.kick)
                    tag_id = next_place_tag(tags, used_tags, measured)
                    glass = choose_glass(measured, tags)
                    if glass is None and done:
                        status = f"all glasses done, {done} placed"
                        print(status)
                        batch = False
                        continue
                    if tag_id is None:
                        status = warn(f"all place tags {sorted(tags)} used"
                                      + (f" after {done} glasses" if done else ", p ignored") + " (c clears them)")
                        batch = False
                        continue
                    if glass is None:
                        status = warn("no glass to pick (none found, or all on place tags), p ignored")
                        batch = False
                        continue
                    if done:
                        print(f"Next glass ({done} placed so far)")
                    print(f"Glass at {glass.x * 1000:.0f}, {glass.y * 1000:.0f} mm "
                          f"({np.hypot(glass.x, glass.y) * 1000:.0f} mm from the base) -> tag {tag_id}")
                    task = SequenceTask(r, c, suction, glass, tags[tag_id], finder.table_z, GLASS_HEIGHT,
                                        sprayer=sprayer, sponge=sponge,
                                        on_placed=lambda tag_id=tag_id: used_tags.add(tag_id))
                # Pace the loop: steps well under SPEED_CMD_TIME apart keep a speedL push
                # continuous, without spinning the CPU between camera frames
                time.sleep(max(0.0, LOOP_PERIOD - (time.time() - loop_start)))
            except Exception as e:
                # Robot fault, lost connection, camera hiccup...: stop and recover in
                # place. Restarting would also cost the gripper's 2 s serial reset.
                traceback.print_exc()
                status = warn(f"error: {e} - stopped, recovering")
                task = None
                recover(r, c, watchdog)
                if freedrive:
                    try:
                        c.endTeachMode()
                    except Exception:
                        pass
                    freedrive = False
                time.sleep(ERROR_RETRY_DELAY)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):   # keep the window alive, allow quit
                    break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if task is not None:
                task.abort()
            if freedrive:
                c.endTeachMode()
            c.speedStop(STOP_DECEL)
            c.stopScript()
        except Exception:
            pass
        gamepad.close()
        suction.close()
        if sprayer is not None:
            sprayer.stop()           # robot already stopped, so waiting for the stroke is fine
        if sponge is not None:
            try:
                sponge.stop()
            except Exception:
                pass
        if servo_bus is not None:
            servo_bus.close()
        grabber.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
