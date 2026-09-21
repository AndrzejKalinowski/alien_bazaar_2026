import rtde_control, rtde_receive
from time import sleep
from pygamepad.gamepads import Gamepad

IP = "127.0.0.1"

def main():
    gamepad = Gamepad()
    gamepad.listen()
    # print(gamepad.buttons.get_list())
    r = rtde_receive.RTDEReceiveInterface(IP)
    print("TCP pose:", r.getActualTCPPose())

    c = rtde_control.RTDEControlInterface(IP)
    
    c.moveJ([0, -1.57, 1.57, -1.57, -1.57, 0], 1.0, 1.0)   # "home"-like pose

    try:
        while True:
            print("valueX:", gamepad.buttons.ABS_X.value, "valueY:", gamepad.buttons.ABS_Y.value)
            p = r.getActualTCPPose()
            p[1] += gamepad.buttons.ABS_X.value/10
            p[2] += gamepad.buttons.ABS_Y.value/-10
            c.moveL(p, 0.25, 0.5)                                   # 10 cm down
            sleep(0.01)
    except (KeyboardInterrupt, SystemExit):
        # Kill gamepad's listening thread
        c.stopScript()
        gamepad.stop_listening()
        # And exit from the program
        exit()


if __name__ == "__main__":
    main()

