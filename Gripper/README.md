# Vacuum gripper controller

PlatformIO project for Seeed Studio XIAO ESP32-C3, using the Arduino framework.

## Wiring

| Signal | XIAO pin | ESP32-C3 GPIO | Active level |
| --- | --- | --- | --- |
| Grip relay input | D1 | GPIO3 | LOW |
| Release relay input | D3 | GPIO5 | LOW |

Connect the relay module ground to the XIAO ground and power the relay module according to its rating. The GPIO outputs provide 3.3 V logic; use relay drivers compatible with that level. Both relay inputs should be held HIGH during reset (for example, with suitable pull-up resistors) if the module must remain off before firmware starts.

## Serial protocol

Use the board's USB serial connection at 115200 baud, 8N1. Send uppercase ASCII commands terminated by LF (`\n`); CRLF also works. Each response ends with LF.

| Command | Response | Meaning |
| --- | --- | --- |
| `STATUS` | `STATE IDLE`, `STATE GRIPPING`, or `STATE RELEASING` | Current relay state |
| `GRIP` | `DONE GRIP` | Sent when the grip relay is switched on |
| `RELEASE` | `DONE RELEASE` | Sent after the release pulse finishes |

`GRIP` stays active until `RELEASE` is received. A repeated `GRIP` while gripping returns `ERR BUSY`. During the release pulse, another `GRIP` or `RELEASE` returns `ERR BUSY`. `RELEASE` is also accepted from `IDLE`. An unknown command returns `ERR UNKNOWN_COMMAND`; a line longer than 31 characters returns `ERR LINE_TOO_LONG`. Empty lines are ignored. The controller does not queue commands.

At startup and after the release pulse, both relays are off (`IDLE`, relay state `00`, GPIO levels `HIGH/HIGH`). `GRIPPING` is relay state `10` (`LOW/HIGH`); `RELEASING` is `01` (`HIGH/LOW`). The grip relay remains energized while `GRIPPING`; make sure the relay and connected load are rated for continuous operation. `RELEASE_PULSE_MS` in `src/main.cpp` sets the release pulse length to 500 ms. A reset or power loss returns the controller to `IDLE`; the grip state is not retained across resets. `DONE` confirms the electrical command, not that an object has been gripped or released.

## Build and upload

Open this directory in VS Code with the PlatformIO extension. Use PlatformIO's Build and Upload actions, or run `pio run` and `pio run -t upload` from the project directory. Open the serial monitor at 115200 baud and select an LF or CRLF line ending.
