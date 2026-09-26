HSBR Web Remote - first version

Files
-----
remote_app.py
templates/remote.html
requirements.txt

Installation on Raspberry Pi
----------------------------
From ~/Documents/HSBR/raspberry_pi :

  python3 -m venv .venv
  source .venv/bin/activate
  pip install -r requirements.txt

If the venv module is not installed:

  sudo apt install python3-venv

Copy the files so that the directory looks like:

  raspberry_pi/
    remote_app.py
    requirements.txt
    templates/
      remote.html

Before starting
---------------
Do NOT leave miniterm or another serial monitor open.
Only one process should own /dev/ttyUSB0.

Check the ESP32 port:

  ls -l /dev/ttyUSB*

Run
---
  cd ~/Documents/HSBR/raspberry_pi
  source .venv/bin/activate
  python3 remote_app.py

Find Raspberry Pi IP:

  hostname -I

Open from iPhone/iPad/PC:

  http://<Raspberry-Pi-IP>:8080/

Initial UI limits
-----------------
Speed : +/-20
Steer : +/-15

Change MAX_SPEED and MAX_STEER near the top of the JavaScript in
templates/remote.html when you are ready to widen the range.

Commands sent to ESP32
----------------------
Joystick   : zj <speed> <steer>
Stand Up   : zj 0 0, then zu1
Stand Down : zj 0 0, then zu0
STOP       : zj 0 0, then zh

Notes
-----
The ESP32 firmware parser terminates a serial command with '\n'.
The Flask reloader is disabled intentionally so the serial port is not
opened by two processes.
