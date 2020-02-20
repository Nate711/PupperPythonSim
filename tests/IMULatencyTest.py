import time
import serial
import threading
import queue

# ser = serial.Serial(
#     port="/dev/cu.usbmodem63711001",
#     baudrate=500000,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     bytesize=serial.EIGHTBITS,
#     timeout=1,
# )

# # while 1:
# #     x = ser.readline().decode("utf8").strip()
# #     print(x)
# #     parsed = x.split(",")
# #     print(parsed)


def get_data_thread(data_queue):
    serial_handle = serial_obj(port="/dev/cu.usbmodem63711001", timeout=1.0)
    while True:
        x = serial_handle.readline().decode('utf8').strip()
        if x:
            parsed = x.split(',')
            data_queue.put(parsed)


def serial_obj(port, timeout):
    return serial.Serial(
        port=port,
        baudrate=500000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=timeout
    )


def test_multithread():
    data_queue = queue.Queue()
    imu_thread = threading.Thread(target=get_data_thread, args=(data_queue,))
    imu_thread.start()

    while True:
        while not data_queue.empty():
            orientation = data_queue.get()
            print(orientation)


def test_inline():
    serial_handle = serial_obj(port="/dev/cu.usbmodem63711001", timeout=0.0001)
    messages = 1
    start = time.time()
    while True:
        x = serial_handle.readline().decode('utf8').strip()
        if x:
            parsed = x.split(',')
            # print(parsed)
            print((time.time()-start)/messages)
            messages += 1
        else:
            pass
            # print("no message")
test_multithread()
# test_inline()