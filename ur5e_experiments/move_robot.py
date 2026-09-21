import rtde_control, rtde_receive
IP = "127.0.0.1"

r = rtde_receive.RTDEReceiveInterface(IP)
print("TCP pose:", r.getActualTCPPose())

c = rtde_control.RTDEControlInterface(IP)
c.moveJ([0, -1.57, 1.57, -1.57, -1.57, 0], 1.0, 1.0)   # "home"-like pose
p = r.getActualTCPPose()
p[2] -= 0.1
c.moveL(p, 0.25, 0.5)                                   # 10 cm down
c.stopScript()