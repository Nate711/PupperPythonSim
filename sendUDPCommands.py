import os
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

# Prints the values for axis0
while True:
    print("running")
    '''
    command = input("Please enter an command (set_velocity, turn_radian, or turn_degrees or break): ")
    msg = {"command": command}
    if command == "set_velocity":
        velocity_x = input("Please enter an x velocity: ")
        velocity_y = input("Please enter an y velocity: ")
        msg["velocity_x"] = velocity_x
        msg["velocity_y"] = velocity_y
    elif command == "turn_radian":
        speed = input("Please enter an turn speed: ")
        radians = input("Please enter the number of radians you wish to turn: ")
        msg["speed"] = speed
        msg["radians"] = radians
    elif command == "turn_degrees":
        speed = input("Please enter an turn speed: ")
        degrees = input("Please enter the number of degrees you wish to turn: ")
        msg["speed"] = speed
        msg["radians"] = degrees
    elif command == "break":
        break
        '''
    msg = {"command": command}
    msg["velocity_x"] = 0.2
    msg["velocity_y"] = 0.2
    print(msg)
    drive_pub.send(msg)
