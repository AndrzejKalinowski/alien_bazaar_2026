# Code audit and improvement plan

**Date:** 2026-09-26 (hackathon day 2 of 3; demo and judging on Sun 27 Sep)
**Scope:** everything in the repo at `656eef1` plus the uncommitted working tree: `ur5e_experiments/` (12 Python files, calibration data), `Gripper/` firmware, `hoverboard_experiments/`, repo hygiene.
**Method:** I read every source file in full, ran a static undefined-name check with the project venv (Python 3.12, ur_rtde 1.6.5, OpenCV 5.0), inspected the saved calibration files, and cross-checked docs against code. **Nothing was run against the robot.** The findings are from reading the code.

Severity: **High** = crashes, or the robot or gripper can misbehave during the demo. **Medium** = wrong results or a trap for teammates. **Low** = hygiene or maintainability.

---

## Summary

The core pipeline (overhead calibration → glass detection → force-guarded pick and place) is well engineered for a hackathon. The geometry is correct, the calibrations are resumable and self-checking, the control loops never block the video, and constants are documented with units. The weak spots are around the edges: one script that no longer starts, **no protection against a stalled Python loop while `speedL` is active**, a gripper-protocol corner case that makes picks fail after a manual grip, and small inconsistencies between the docs and the code.

For **tomorrow's demo**, do items 1–4 and the "demo hardening" list at the end. Everything else can wait.

---

## Findings

### High

**1. `gamepad_robot_teleop.py` crashes on start: `NameError: GAMEPAD_KEYS`**
[ur5e_experiments/gamepad_robot_teleop.py:84](ur5e_experiments/gamepad_robot_teleop.py#L84). The uncommitted refactor to `GamepadControl` removed the old button handling but never defined the key map. `DEADZONE` / `apply_deadzone` are now unused too. *Fixed on branch `fix/audit`.*
*Fix:* add `GAMEPAD_KEYS = {"BTN_START": "h", "BTN_EAST": "g", "BTN_WEST": "r"}` (the old behaviour) and delete the dead helpers.

**2. No motion watchdog: if the Python loop stalls, the robot keeps moving** *Fixed on branch `fix/audit`: `robot_watchdog.py` (5 Hz, not 10 Hz, for margin against slow frames), `read_frame()` timeout.*
`speedL` keeps executing the last velocity until a new command or `speedStop` arrives. `follow_april_tag.py` handles a camera timeout, but:
- [find_glasses.py:238](ur5e_experiments/find_glasses.py#L238) `read_frame()` loops **forever** if the camera stops delivering frames (USB glitch or unplug). It is used by `find_glasses.py` and `pick_place_glasses.py` in the main loop, *including during `PickPlaceTask.push_down()`, which is a `speedL` descent*, and during jogging.
- A Python exception outside the `try` or a debugger breakpoint has the same effect.

Today the only backstops are the force sensor (a protective stop on hard contact) and the e-stop.
*Fix (small):* call `rtde_c.setWatchdog(10.0)` after connecting and `rtde_c.kickWatchdog()` once per loop iteration. ur_rtde 1.6.5 has both. The robot then stops by itself if kicks stop arriving for about 100 ms. Also give `read_frame()` a timeout (e.g. 0.5 s) that raises, so the `finally` block runs `speedStop`.

**3. Pick after a manual grip: narrower than first reported** *Corrected and fixed on branch `fix/audit`.*
The firmware answers `GRIP` with `ERR BUSY` while it is `GRIPPING` ([Gripper/src/main.cpp:171](Gripper/src/main.cpp#L171)) and changes nothing: the vacuum stays on with the first `GRIP`'s atmospheric baseline. The original claim here ("no new `GRIP …` line arrives") was wrong for the common cases: `updateHolding()` sends `GRIP OK` / `GRIP LOST` on every change of the held state, independent of `gripResultSent`. So **g** then **p**, or a retry after "glass lost while carrying", still gets `GRIP OK` when the cup seals on the new glass.

The real bug was that `Suction.grip()` always cleared `_grip_result`, even when the controller rejected the `GRIP`. That broke two cases, both ending in a 6 s timeout, release and abort:
- no pressure sensor: `GRIP UNKNOWN` is sent only once, at the first `GRIP`;
- something already held when **p** is pressed: `holding` does not change, so no new `GRIP OK`.

*Fix (done):* `grip()` no longer clears the result. It is cleared on `DONE GRIP` (the controller really started a new session with a fresh baseline) and on `release()`. The firmware is unchanged.

**4. Jogging has no ceiling or floor guard outside `follow_april_tag.py`** *Fixed on branch `fix/audit`: `Jogger(c, gamepad, min_z, max_z).update(tcp_pose)` caps Z with `limit_z_speed()` (also used by `follow_april_tag`, which gains the floor guard). `pick_place_glasses` and teleop refuse a home above the ceiling, and `pick_place_glasses` aborts a task more than `CEILING_MARGIN` above it. `find_glasses` imports `MIN_TCP_Z` instead of its own copy.*
`follow_april_tag.py` caps upward jog speed near `MAX_TCP_Z`, but `Jogger` ([gamepad_jog.py:98](ur5e_experiments/gamepad_jog.py#L98)), used by `find_glasses.py`, `pick_place_glasses.py` and `gamepad_robot_teleop.py`, sends raw stick speeds with no Z limits. `HomeTask` in [pick_place_glasses.py:126](ur5e_experiments/pick_place_glasses.py#L126) also skips the "home above ceiling" check that `follow_april_tag.HomeTask` has.
*Fix:* move the ceiling cap (and a `MIN_TCP_Z` floor cap) into `Jogger.update()` by passing it the current TCP pose, so every script gets the same limits.

### Medium

**5. Gripper README said the release pulse was 500 ms; the code uses 1500 ms**. *Fixed in this pass; on branch `fix/audit` all docs checked (1.5 s everywhere), and Gripper/README.md now names the host constants that mirror the pulse length.* [Gripper/src/main.cpp:13](Gripper/src/main.cpp#L13) is `RELEASE_PULSE_MS = 1500`, and `suction.py` / `pick_place_glasses.py` (`RELEASE_TIME = 1.7`) rely on 1.5 s.

**6. The firmware cannot detect a failed pressure reading mid-run** *Fixed on branch `fix/audit`: chip-ID check after every reading + 300–1100 hPa range → `NAN`. Also fixed: a `NAN` while gripping used to set `holding = false` and send a false `GRIP LOST`; now the held state is kept and `GRIP UNKNOWN` is sent after 500 ms. A sensor missing at boot is re-probed every 2 s. Built with `pio run`, not flashed.*
[Gripper/README.md](Gripper/README.md) says `HOLD`/`PRESSURE` return `ERR NO_SENSOR` "if … a reading fails". In the code, `NaN` only happens when the sensor is missing at boot. `Adafruit_BMP085::readPressure()` returns an integer and does not report I2C errors. A loose I2C wire during the demo therefore produces garbage pressure, which can trigger a false `GRIP OK` or `GRIP LOST`.
*Fix:* in `samplePressure()`, treat values outside 300–1100 hPa (the BMP180 range) as `NAN`. Optionally re-probe the chip ID every few seconds.

**7. The rejected-circle overlay goes stale** *Fixed on branch `fix/audit`.*
[find_glasses.py:338](ur5e_experiments/find_glasses.py#L338): `detect()` returns early when `HoughCircles` finds nothing, *before* `self.rejected = []`. The red "rejected" circles from the last frame that had detections stay on screen. This is misleading while tuning the trackbars.
*Fix:* reset `self.rejected` at the top of `detect()`.

**8. `RIM_HEIGHT` means two different things** *Fixed on branch `fix/audit`: `pick_place_glasses.GLASS_HEIGHT` (same 0.075 m), `GlassFinder.load(rim_height=...)`.*
In `find_glasses.py` it is the height of the circle seen for *upright* glasses. `pick_place_glasses.py` reuses `fg.RIM_HEIGHT` as the *full height of an upside-down glass* (where the cup lands, and the carry height is computed as 2 × `RIM_HEIGHT` + clearance). Tuning it for one script silently changes the other.
*Fix:* give `pick_place_glasses.py` its own `GLASS_HEIGHT` and build the `GlassFinder` with it (add a `rim_height` parameter to `GlassFinder.load`).

**9. The place target defaults to "lowest tag id in view"** *Partly done on branch `fix/audit`: the chosen id is drawn next to the place marker, and p warns (without refusing) when several tags are in view. `PLACE_TAG_ID` stays `None` on purpose: the demo tag's id is not known in advance and may change.*
[pick_place_glasses.py:51](ur5e_experiments/pick_place_glasses.py#L51) `PLACE_TAG_ID = None`. The overhead calibration uses the same tag family, so a calibration tag left on the table can become the place target.
*Fix:* set an explicit id for the demo, and draw the id next to the "place" marker.

**10. Camera indices are fragile and documented inconsistently** *Fixed on branch `fix/audit`: docstring says `--camera 2`, new `calibrate_camera.py --list-cameras` shows every index labelled with its role. Picking cameras by device name is still open.*
`OVERHEAD_CAMERA_INDEX = 2` in [find_glasses.py:114](ur5e_experiments/find_glasses.py#L114), but [calibrate_camera.py:37](ur5e_experiments/calibrate_camera.py#L37) says `--camera 1` for the overhead camera. On Windows the DirectShow indices change when cameras are re-plugged. If the two cameras swap, each gets the other's calibration, and nothing complains, because both are 1280×720.
*Fix:* fix the docstring. Before the demo, verify which camera is which (e.g. a `--list-cameras` helper that shows every index). Longer term, pick cameras by device name.

**11. The saved detection settings effectively disable the colour filter** *Checked on branch `fix/audit`: not a bug, no change.*
`detection_settings.json` has `"max saturation": 211` (the code default is 60), so almost nothing is rejected for colour. But it also has `"min brightness": 139` (default 0 = off), which this item first missed: circles must have a bright rim, which is how glass rims look. So the filtering was moved from colour to brightness, most likely on purpose for the current glasses and lighting. The file is runtime tuning data and was left as it is. Re-check both trackbars on the demo table (the `S.. V..` values next to each circle).

**12. `bus_servos.py` docs contradict the servo ID constants**
The constants are `ROTATOR_ID = 2` and `SPRAYER_ID = 1`, but the docstring says "give the sprayer its own ID first: `set-id 1 2`" and the usage example builds `rotator = BusServo(bus, 1)`, `sprayer = … BusServo(bus, 2)`. Following the docstring gives the servos the wrong roles.

**13. `pick_place_glasses.py` has no recovery after a robot fault**
`follow_april_tag.py` catches exceptions from robot calls, stops, and calls `reuploadScript()`. `pick_place_glasses.py` lets any RTDE exception (e.g. after a protective stop) end the program. During a live demo, recovering in place is much faster than restarting the script and waiting for the gripper's 2 s serial reset.

### Low

**14. PlatformIO build output is committed.** `git ls-files Gripper/.pio` shows **102 files (~26 MB)**, including `firmware.elf`, `.o` and `.a` files. `.pio` is ignored now, but the files were committed before that. *Fix:* `git rm -r --cached Gripper/.pio` and commit.

**15. There is no `requirements.txt`.** The dependencies exist only inside the local `.venv`. A teammate cannot reproduce the environment. See README §4.1 for the exact versions; put them in `ur5e_experiments/requirements.txt`.

**16. The calibration data is untracked.** `*.npz`, `detection_area.json`, `detection_settings.json` and `overhead_calibration_points.json` are untracked, and `detection_settings.json.tmp` (a leftover atomic-write temp file) is sitting in the tree. One `git clean` would lose a good calibration. *Fix:* commit the current calibration as a "demo baseline" (it is small and specific to this setup, which is fine for a hackathon repo), and add `*.tmp` and `*.bak` to `.gitignore`.

**17. Configuration is duplicated.** `IP` is defined in 4 files, `HOME_Q` in 2, `MIN_TCP_Z` in 2, `SPEED_CMD_TIME` in 3 (with the same values, for now). COM ports are hard-coded in 4 files. *Fix:* a single `config.py` (or environment variables / CLI flags for ports and camera indices).

**18. `follow_april_tag.py` is used as a library but has import side effects.** Four scripts import it for constants and helpers. Importing it:
- requires `ur_rtde`, even for `calibrate_camera.py`, which never touches the robot;
- loads `hand_eye.npz` and prints a message at import time (`T_TCP_CAM = tcp_to_camera_matrix()`).

*Fix:* move `pose_to_matrix`, `Camera`, `load_intrinsics`, `TagDetector`, `connect_suction` / `NoSuction` and the shared constants into a `common.py` / `robot.py` with no side effects.

**19. Two task frameworks do the same job.** `follow_april_tag.PickTask` is a stage-string state machine; `pick_place_glasses.Task` is a generator run one step per frame. The generator version is shorter and easier to extend. Standardise on it.

**20. Smaller issues in `follow_april_tag.py`.** `import math` is unused. When the camera times out, the loop `continue`s before `cv2.waitKey`, so the window freezes and **q** does not work until frames return (Ctrl+C still works).

**21. Legacy scripts.** `gamepad_robot_move.py` issues *blocking* `moveL`s of up to 10 cm per loop from raw stick values, and `move_robot.py` targets URSim. They are fine as history but confusing next to the real tools. Move them to `ur5e_experiments/legacy/` or delete them.

**22. Hoverboard.** The two teleop scripts use opposite throttle signs: `gamepad_hoverboard_teleop.py` negates `ABS_Y` and `gamepad_xiao_teleop.py` does not. Check that forward is forward on both. `xiao_send_pwm.ino` appends to a `String` with no length limit, so a noisy line without `\n` grows the heap. Cap it at about 32 characters.

**23. Docs out of date.** The old root README said "gripper RobotiQ". The actual gripper is the custom suction cup (fixed in the new README). `ur5e_experiments/README.md` has the typo "ue5 docs". The brief's file name `alien-bazaar-2026-brief (1).md` contains a space and "(1)", which is awkward to link or type. Consider renaming it to `docs/brief.md`.

**24. No automated tests.** Several pieces are pure functions and easy to test offline:
- `pixel_to_plane`, `tag_centers`, `solve_camera_pose` (synthetic camera);
- `rotvec_between` / `rotation_error`;
- `park_martin` (synthetic AX = XB with a known X);
- the `Suction` line parser (fake serial);
- the `bus_servos` packet checksum and sign-magnitude helpers;
- `bipropellant_serial` framing (compare with the reference implementation).

A `pytest` file running in under a second would catch refactor slips like #1.

---

## What is already good (keep doing this)

- **Every script has a docstring** that explains the method, the setup and the controls. This is the best documentation in the repo.
- **Constants at the top, with units** in comments.
- **Calibrations are self-validating**: the per-point mm error for the overhead pose, the tag-position spread for hand-eye, the reprojection error for the intrinsics.
- **Crash-safe persistence**: atomic `os.replace` writes and resumable overhead calibration.
- **Non-blocking control loops**: tasks advance once per frame, so the video stays live and **manual override** (touching a stick) aborts a task immediately.
- **Force-guarded approaches** with a max-overshoot fallback, and grips are confirmed by measured vacuum instead of hope.
- **The gripper firmware is a clean, non-blocking state machine**. Its thresholds come from a measured 10-cycle test, and its protocol is documented precisely.

---

## Suggestions for this stage of development

### Before the demo (today / tomorrow morning)

1. **Fix #1–#4.** Each is a few lines. #2 (watchdog) is the one most likely to hurt you live (#3 turned out to be narrower, see above).
2. **Freeze and back up the calibration.** Commit the `.npz` / `.json` files, tag the commit (`git tag demo-baseline`), and copy them to a USB stick. After that, **do not move the overhead camera or change the TCP**.
3. **Pre-flight script** (`preflight.py`, about 50 lines). It prints a pass/fail list:
   - robot reachable and not protective-stopped;
   - TCP offset as expected;
   - gripper answers `STATUS`, and `PRESSURE` is in the 950–1050 hPa range;
   - both cameras open at 1280×720, and the overhead camera sees the place tag;
   - calibration files present;
   - gamepad detected.

   Run it before every demo slot.
4. **Set `PLACE_TAG_ID`** and remove any stray tags from the table.
5. **Continuous mode for the show.** Add a key that repeats pick → place until no glasses are left, with a small stack or line of place spots instead of a single tag. A robot that keeps working is far more convincing to a jury than single button presses.
6. **Rehearse the failure paths once**: unplug the gripper USB, cover the camera, trigger a protective stop. Know what the screen shows and how to recover in under 30 s.
7. **Collaboration hook** (an explicit judging criterion). Expose a tiny HTTP or MQTT endpoint, e.g. `GET /glasses` (positions) and `POST /place` (a request from another team, such as Robo Bar or Boróweczki handing over glasses). A 30-line `http.server` thread that reads `finder.measure()` results is enough to demo interoperability.

### After the hackathon (if the project continues)

- **Restructure into a package**: `rabyte/{config,robot,camera,vision,gripper,tasks}.py`, with the scripts as thin entry points. This removes the import side effects (#18) and the duplicated config (#17).
- **One task framework** (the generator style), with an explicit, logged state per step. Use `logging` instead of `print`, with timestamps, so failed demo runs can be analysed afterwards.
- **Tests + CI** for the pure-math and protocol pieces (#24), and a GitHub Action running `pytest` and `pio run`.
- **Vision robustness**: the Hough + HSV approach depends on lighting. Next steps, in order of effort: a fixed LED ring or diffuser and a fixed exposure (turn off webcam auto-exposure), then depth (OAK-D / RealSense) to separate glasses by height, then a small learned detector if the scene gets more varied.
- **Gripper**: add the pressure-range sanity check (#6). Optionally add a heartbeat/timeout mode that drops the vacuum if the host has been silent for a long time, but only if you decide that dropping a held object is safer than holding it.
- **Use the UR's own safety configuration** (safety planes, tool orientation limits, reduced mode) as the real safety boundary, and keep the Python limits as a second layer.
