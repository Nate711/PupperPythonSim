import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
start = time.time()
n = 100
for i in range(n):
    for a in range(4, 16):
        kit.servo[a].angle = i % 180
    # time.sleep(0.001)
    if i % 10 == 0:
        print(i)
end = time.time()
print("Seconds per 12-actuator update: ", (end - start) / 100.0)
