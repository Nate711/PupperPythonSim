import serial
import numpy as np


class IMU:
    def __init__(self, port, baudrate=500000, timeout=0.0001):
        self.serial_handle = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout,
        )
        self.last_quat = np.array([1, 0, 0, 0])

    def flush_buffer(self):
        self.serial_handle.reset_input_buffer()()

    def read_orientation(self):
        """Reads quaternion measurements from the Teensy until none are left. Returns the last read quaternion.
        
        Parameters
        ----------
        serial_handle : Serial object
            Handle to the pyserial Serial object
        
        Returns
        -------
        np array (4,)
            If there was quaternion data to read on the serial port returns the quaternion as a numpy array, otherwise returns the last read quaternion.
        """
        while True:
            x = self.serial_handle.readline().decode("utf").strip()
            if x:
                parsed = x.split(",")
                assert len(parsed) == 4
                self.last_quat = np.array(parsed, dtype=np.float64)
            else:
                return self.last_quat
