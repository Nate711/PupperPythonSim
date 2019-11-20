import os
import time
from UDPComms import Publisher
import signal

drive_pub = Publisher(8870)


# prevents quiting on pi when run through systemd
def handler(signum, frame):
    print("GOT singal", signum)


signal.signal(signal.SIGHUP, handler)

# those two lines allow for running headless (hopefully)
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.putenv('DISPLAY', ':0.0')
time.sleep(5)
# Prints the values for axis0
command = input("Please enter an command (set_velocity, turn_radian, or turn_degrees or break): ")
while True:
    if command == "set_velocity" or command[:3] == "set":
        msg = {"command": "set_velocity"}
        velocity_x = input("Please enter an x velocity: ")
        velocity_y = input("Please enter an y velocity: ")
        msg["velocity_x"] = float(velocity_x)
        msg["velocity_y"] = float(velocity_y)
    elif command == "turn_radian" or command[:4] == "turn" and "radian" in command:
        msg = {"command": "turn_radians"}
        speed = input("Please enter an turn speed: ")
        radians = input("Please enter the number of radians you wish to turn: ")
        msg["speed"] = float(speed)
        msg["radians"] = float(radians)
    elif command == "turn_degrees" or command[:4] == "turn" and "degrees" in command:
        msg = {"command": "turn_degrees"}
        speed = input("Please enter an turn speed: ")
        degrees = input("Please enter the number of degrees you wish to turn: ")
        msg["speed"] = float(speed)
        msg["radians"] = float(degrees)
    elif command == "break":
        break
    #msg = {"command": "set_velocity", "velocity_x": 0.1, "velocity_y": 0.0}
    print(msg)
    drive_pub.send(msg)
    #time.sleep(2)
