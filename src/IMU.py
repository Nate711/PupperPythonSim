import serial
import numpy as np


def create_imu_handle(imu_params):
    return serial.Serial(
        port=imu_params.port,
        baudrate=imu_params.baudrate,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=imu_params.timeout,
    )


def read_orientation(serial_handle):
    """Reads quaternion measurements from the Teensy until none are left. Returns the last read quaternion.
    
    Parameters
    ----------
    serial_handle : Serial object
        Handle to the pyserial Serial object
    
    Returns
    -------
    np array (4,) or None
        If there was quaternion data to read on the serial port returns the quaternion as a numpy array, otherwise returns None
    """
    quat = None
    while True:
        x = serial_handle.readline().decode("utf").strip()
        if x:
            quat = np.array(x.split(","), dtype=np.float64)
        else:
            return quat

