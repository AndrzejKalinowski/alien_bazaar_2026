## Learning ur5 programming

Run robot interface simulation in WSL (you need wsl and docker)
```
wsl -- sudo docker run --rm -it --name ursim -p 5900:5900 -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004 -v ~/ursim/programs:/ursim/programs universalrobots/ursim_e-series
```
web interface:
http://localhost:6080/vnc.html

Point `IP` in `follow_april_tag.py` at `127.0.0.1` to run the scripts against it.

## ur_rtde docs
https://sdurobotics.gitlab.io/ur_rtde/index.html