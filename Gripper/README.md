# Vacuum gripper controller

PlatformIO project for Seeed Studio XIAO ESP32-C3, using the Arduino framework.

## Wiring

| Signal | XIAO pin | ESP32-C3 GPIO | Active level |
| --- | --- | --- | --- |
| Grip relay input | D1 | GPIO3 | LOW |
| Release relay input | D3 | GPIO5 | LOW |
| Pressure sensor SDA | D4 | GPIO6 | I2C |
| Pressure sensor SCL | D5 | GPIO7 | I2C |

Connect the relay module ground to the XIAO ground and power the relay module according to its rating. The GPIO outputs provide 3.3 V logic; use relay drivers compatible with that level. Both relay inputs should be held HIGH during reset (for example, with suitable pull-up resistors) if the module must remain off before firmware starts.

The pressure sensor is the BMP180 barometer (I2C address 0x77, chip ID 0x55) on the Waveshare 10 DOF IMU Sensor; the MPU9255 on the same board (0x68) is not used. The product page lists a BMP280, but the board in use has a BMP180. Power the board from the XIAO 3V3 pin (or 5V) and GND. The sensor must sit inside the vacuum line (in a sealed chamber or tee between the valve and the suction cup), so it reads the pressure in the cup, not the room pressure. The BMP180 measures 300–1100 hPa; stronger vacuum reads as the bottom of its range, which is still enough to detect a held object.

## Serial protocol

Use the board's USB serial connection at 115200 baud, 8N1. Send uppercase ASCII commands terminated by LF (`\n`); CRLF also works. Each response ends with LF.

| Command | Response | Meaning |
| --- | --- | --- |
| `STATUS` | `STATE IDLE`, `STATE GRIPPING`, or `STATE RELEASING` | Current relay state |
| `GRIP` | `DONE GRIP`, later `GRIP OK` or `GRIP FAIL` | `DONE GRIP` when the grip relay is switched on; then the pick result (see below) |
| `RELEASE` | `DONE RELEASE` | Sent after the 1.5 s release pulse (`RELEASE_PULSE_MS`) finishes |
| `HOLD` | `HOLD YES` or `HOLD NO` | Whether an object is held (vacuum reached) |
| `PRESSURE` | `PRESSURE <hPa>`, or `PRESSURE <hPa> BASE <hPa> VACUUM <hPa>` while gripping | Raw sensor reading for calibration |

While `GRIPPING`, the controller also sends these lines on its own:

| Message | Meaning |
| --- | --- |
| `GRIP OK` | The object is held: the vacuum reached `HOLD_ON_THRESHOLD_HPA` |
| `GRIP FAIL` | Nothing was held within `GRIP_CONFIRM_TIMEOUT_MS` (8 s) after `GRIP`. The grip relay stays on; if the object seals later, `GRIP OK` still follows |
| `GRIP LOST` | A held object was lost: the vacuum fell below `HOLD_OFF_THRESHOLD_HPA` |
| `GRIP UNKNOWN` | Sent right after `DONE GRIP` when the pressure sensor is missing, or while `GRIPPING` once the sensor has given no valid reading for 500 ms (`SENSOR_LOST_MS`). No `GRIP FAIL` follows then |

These messages can arrive between the reply lines of other commands, so a host should match replies by their first word.

`GRIP` stays active until `RELEASE` is received. A repeated `GRIP` while gripping returns `ERR BUSY` and changes nothing: the vacuum stays on with the first `GRIP`'s baseline, and `GRIP OK` / `GRIP LOST` still follow on every change of the held state. During the release pulse, another `GRIP` or `RELEASE` returns `ERR BUSY`. `RELEASE` is also accepted from `IDLE`. An unknown command returns `ERR UNKNOWN_COMMAND`; a line longer than 31 characters returns `ERR LINE_TOO_LONG`. Empty lines are ignored. The controller does not queue commands.

At startup and after the release pulse, both relays are off (`IDLE`, relay state `00`, GPIO levels `HIGH/HIGH`). `GRIPPING` is relay state `10` (`LOW/HIGH`); `RELEASING` is `01` (`HIGH/LOW`). The grip relay remains energized while `GRIPPING`; make sure the relay and connected load are rated for continuous operation. `RELEASE_PULSE_MS` in `src/main.cpp` sets the release pulse length to 1500 ms. The host relies on this length: `RELEASE_TIMEOUT` (3 s) in `ur5e_experiments/suction.py` and `RELEASE_TIME` (1.7 s) in `ur5e_experiments/pick_place_glasses.py`. Change them together. A reset or power loss returns the controller to `IDLE`; the grip state is not retained across resets. `DONE` confirms the electrical command, not that an object has been gripped or released; use `GRIP OK`/`GRIP FAIL` or `HOLD` for that.

Just before the grip relay is switched on, the firmware stores the current pressure as the baseline. The sensor is sampled every 50 ms; each reading blocks the loop for about 13 ms. `HOLD` returns `HOLD YES` only while `GRIPPING`, once the pressure has dropped at least `HOLD_ON_THRESHOLD_HPA` (180 hPa) below that baseline. It then stays `HOLD YES` until the drop falls below `HOLD_OFF_THRESHOLD_HPA` (120 hPa), for example when the object slips off; otherwise it returns `HOLD NO`. The thresholds come from a 10-cycle test: with an open cup the vacuum settles at 71–99 hPa, and with a held object it reaches 213–395 hPa. When the object seals, the drop passed 180 hPa 0.9–5.4 s after `GRIP` (1–3 s in most cycles), which is why `GRIP FAIL` waits 8 s. To recalibrate, send `GRIP` and read `PRESSURE` with the cup open and then with an object sealed on it, and set the thresholds between the two `VACUUM` levels in `src/main.cpp`. If the sensor is not detected at startup or a reading fails, `HOLD` and `PRESSURE` return `ERR NO_SENSOR`; the relays keep working without the sensor. The library cannot report I2C errors (a loose wire gives garbage values), so a reading counts as failed unless the chip still answers with its ID (0x55) afterwards and the value is inside the BMP180's 300–1100 hPa range. A failed reading while `GRIPPING` keeps the held state, so a glitch never causes a false `GRIP LOST`; after 500 ms without a valid reading `GRIP UNKNOWN` is sent once. A sensor missing at startup is probed again every 2 s (`SENSOR_RETRY_MS`), so fixing the wire brings it back without a reset.

## Build and upload

Open this directory in VS Code with the PlatformIO extension. Use PlatformIO's Build and Upload actions, or run `pio run` and `pio run -t upload` from the project directory. Open the serial monitor at 115200 baud and select an LF or CRLF line ending.
