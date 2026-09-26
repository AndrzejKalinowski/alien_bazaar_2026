# CLAUDE.md

Guidance for AI coding assistants working in this repo. Human-facing docs are in [README.md](README.md). Known bugs and the roadmap are in [AUDIT.md](AUDIT.md).

## What this is

Hackathon code (Alien Bazaar 2026, team Rabyte) for a **UR5e robot arm with a custom suction gripper**. It is controlled from a Windows laptop in Python over RTDE (`ur_rtde`). Main demo: [ur5e_experiments/pick_place_glasses.py](ur5e_experiments/pick_place_glasses.py). An overhead camera finds upside-down glasses, and the robot picks one and places it on an AprilTag. It is experimental code under time pressure: prefer small, targeted changes over refactors unless asked.

## This code moves real hardware

- **Never run** a script that connects to the robot (`192.168.1.20`), the gripper or the servos unless the user explicitly asks. Anything that imports `rtde_control` and constructs `RTDEControlInterface` can move the arm. Offline checks are always fine: `compileall` (below), pure-function tests. Note that `--help` still imports the module, and some modules connect or open ports only inside `main()`, so check first.
- **Do not loosen safety limits silently.** This covers `MIN_TCP_Z`, `MAX_TCP_Z`, `MAX_OVERSHOOT`, `CONTACT_FORCE`/`PLACE_FORCE`, speed and acceleration constants, `CAMERA_TIMEOUT` and tilt checks. If a change needs it, say so explicitly.
- **Always pass a non-zero time to `speedL`** (`SPEED_CMD_TIME = 0.02`). `time=0` makes the robot protective-stop (C271A1). `speedL` keeps moving until the next command or `speedStop`. Never add code that blocks the main loop while a `speedL` is active; call `jogger.stop()` / `speedStop()` first.
- **Motion watchdog:** robot main loops create `RobotWatchdog(rtde_c)` right before the loop and call `watchdog.kick()` once per iteration (False = robot was stopped, abort the task). No blocking `moveJ`/`moveL` in those loops (use `async=True`), nothing else that blocks > 0.2 s without kicking, and call `watchdog.arm()` after every `reuploadScript()`.
- Every exit path must stop motion: keep the `try/finally` blocks that call `speedStop` / `stopL` / `stopScript`.

## Layout

- `ur5e_experiments/`: all robot code. Flat scripts, no package. Run from this directory (imports are sibling-module imports).
  - `pick_place_glasses.py`: main demo (generator-based `Task`s stepped once per video frame).
  - `find_glasses.py`: `GlassFinder` (Hough circles + HSV rim filter + back-projection to base frame), `AreaEditor`, overhead-camera calibration (`--calibrate`). It is both a tool and a library.
  - `follow_april_tag.py`: wrist-camera tag picker **and the de facto shared module**: `IP`, `HOME_Q`, `MIN/MAX_TCP_Z`, `TAG_DICTIONARY`, `TAG_SIZE`, `pose_to_matrix`, `Camera`, `TagDetector`, `connect_suction`. Importing it has side effects (it loads `hand_eye.npz` and needs `ur_rtde`).
  - `gamepad_jog.py`: `GamepadControl` (button → key edges, stick → speed vector), `Jogger`.
  - `robot_watchdog.py`: `RobotWatchdog`, the RTDE watchdog that stops the robot when a main loop stalls.
  - `suction.py`: host driver for the gripper serial protocol.
  - `hand_eye_calibration.py`, `calibrate_camera.py`: calibration tools that write the `.npz` files.
  - `bus_servos.py`: Feetech STS bus servos (not integrated with the robot yet).
  - `gamepad_robot_move.py`, `move_robot.py`: legacy experiments.
- `Gripper/`: PlatformIO firmware (XIAO ESP32-C3, Arduino) for the suction controller. [Gripper/README.md](Gripper/README.md) is the protocol spec.
- `hoverboard_experiments/`: gamepad → hoverboard (bipropellant protocol), independent of the robot.

## Commands

```powershell
# Python env (Windows; venv is ur5e_experiments/.venv, Python 3.12)
ur5e_experiments\.venv\Scripts\Activate.ps1
pip install ur_rtde==1.6.5 opencv-python==5.0.0.93 numpy pyserial cobs inputs pygamepad

# Offline syntax check of everything
python -m compileall -q -x "\.venv" ur5e_experiments hoverboard_experiments

# Firmware (from Gripper/)
pio run                 # build
pio run -t upload       # flash (user only)
```

There are no tests yet. For robot logic without hardware, URSim can run in Docker/WSL (see [ur5e_experiments/README.md](ur5e_experiments/README.md)); point `IP` at `127.0.0.1`.

## Conventions (match these)

- **Module docstring = the documentation.** Each script starts with a long docstring: how it works, setup, keys and gamepad buttons, and the `Requires:` line. When you change behaviour, keys or constants, **update that docstring**, and README.md if it is affected.
- **Configuration is module-level UPPER_CASE constants** at the top of the file, grouped under `# --- section ---` headers, **each with its unit in a comment** (`# m`, `# m/s`, `# N`, `# s`, `# px`, `# deg`). No config files for code parameters. The JSON files are only for data the user edits at runtime (detection area, trackbar settings, calibration progress).
- **Units:** metres, seconds and radians internally. Degree constants end in `_DEG`. UR poses are `[x, y, z, rx, ry, rz]` with an **axis-angle** rotation. Convert with `pose_to_matrix()` / `cv2.Rodrigues`. Homogeneous transforms are named `T_<to>_<from>` (e.g. `T_base_cam`, `T_tcp_cam`, `T_cam_tag`) and compose left to right: `T_base_tag = T_base_tcp @ T_tcp_cam @ T_cam_tag`.
- **UI loop pattern:** one loop per video frame: read frame → detect → step the task (`task.update()`) → draw → `cv2.imshow` → read keys (`read_keys()` merges gamepad button edges and `cv2.waitKey`). Long actions are tasks that return quickly each step. Gamepad buttons are mapped to the same key characters as the keyboard (`GAMEPAD_KEYS = {"BTN_NORTH": "p", ...}`).
- **Manual override:** any stick input aborts the running task. Keep this in new tasks.
- **Persisted files** are written atomically (write `*.tmp`, then `os.replace`), and paths are built relative to `os.path.dirname(__file__)`.
- Comments explain *why* (hardware quirks, measured values, protocol details), not what. Keep that style and density.
- Firmware: `namespace {}` for file-local state, `constexpr` constants with measured justification in comments, non-blocking `loop()` built from `update*()` functions.

## Cross-file contracts (change together)

- **Gripper serial protocol:** `Gripper/src/main.cpp` ↔ `ur5e_experiments/suction.py` ↔ `Gripper/README.md`. Timings that are mirrored in Python: `RELEASE_PULSE_MS` = 1500 ↔ `RELEASE_TIMEOUT`, and `RELEASE_TIME` in `pick_place_glasses.py`. `GRIP_CONFIRM_TIMEOUT_MS` = 8000 ↔ `GRIP_RESULT_TIMEOUT` and the tasks' `GRIP_CONFIRM_TIMEOUT` (6 s). The unsolicited `GRIP OK/FAIL/LOST/UNKNOWN` lines are parsed in `Suction._read_lines`.
- **Calibration files** (all 1280×720, all relative to the TCP set on the pendant at the suction-cup tip): the `camera_calibration.npz` and `overhead_camera_calibration.npz` keys `camera_matrix`, `dist_coeffs`, `image_size`; the `hand_eye.npz` key `T_tcp_cam`; the `overhead_camera_pose.npz` keys `T_base_cam`, `table_z`, `image_size`, `pixels`, `base_points`. Do not change these keys without updating every reader.
- **`find_glasses.py` is imported by `pick_place_glasses.py`** (`fg.setup_window`, `fg.read_trackbars`, `fg.draw_overlay`, `fg.RIM_HEIGHT`, `GlassFinder`, `AreaEditor`, `tag_centers`, `pixel_to_plane`). Keep those names stable.
- **`follow_april_tag.py` constants are imported by 4 scripts.** Renaming them breaks the others.

## Don't

- Don't commit or delete the calibration data (`*.npz`, `detection_*.json`, `overhead_calibration_points.json`) without asking. It takes robot time to recreate.
- Don't add `Gripper/.pio/` build output to commits (it is ignored, but old artifacts are still tracked; see AUDIT.md #14).
- Don't introduce ROS or other heavy frameworks. The stack is deliberately `ur_rtde` + OpenCV + pyserial.
- Don't assume COM ports or camera indices. They are machine-specific and set in the constants (see README §14).
