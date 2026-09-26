"""
Print the UR5e's built-in force/torque sensor readings.

Reads getActualTCPForce() over RTDE: the wrench at the TCP, expressed in the
base frame, [Fx, Fy, Fz] in N and [Tx, Ty, Tz] in Nm, plus |F|. Only the
receive interface is used, so this script never moves the robot and can run
alongside the pendant or another script.

The sensor drifts; values are relative to the last zeroFtSensor() (which the
pick tasks call before each contact move), so expect a few N offset at rest.

Usage: python read_force.py        (Ctrl+C to quit)

Requires: pip install ur_rtde
"""

import time

import numpy as np
import rtde_receive

IP = "192.168.1.20"  # same as follow_april_tag.IP (not imported: that module has side effects)

PRINT_PERIOD = 0.1  # s


def main():
    r = rtde_receive.RTDEReceiveInterface(IP)
    print("    Fx      Fy      Fz   |    Tx     Ty     Tz   |   |F|")
    try:
        while True:
            w = r.getActualTCPForce()
            print(f"{w[0]:7.2f} {w[1]:7.2f} {w[2]:7.2f} | "
                  f"{w[3]:6.3f} {w[4]:6.3f} {w[5]:6.3f} | "
                  f"{np.linalg.norm(w[:3]):6.2f}")
            time.sleep(PRINT_PERIOD)
    except KeyboardInterrupt:
        pass
    finally:
        r.disconnect()


if __name__ == "__main__":
    main()
