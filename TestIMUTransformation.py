from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat
from src.IMU import read_orientation, create_imu_handle
from src.PupperConfig import IMUParams

imu_params = IMUParams("/dev/cu.usbmodem63711001")
imu_handle = create_imu_handle(imu_params)
imu_handle.reset_input_buffer()

while True:
    quat_orientation = read_orientation(imu_handle)
    if quat_orientation is not None:
        # q_inv = qconjugate(quat_orientation)
        # (yaw, pitch, roll) = quat2euler(q_inv)
        (yaw, pitch, roll) = quat2euler(quat_orientation)
        print(round(roll,3), round(pitch,3), round(yaw,3))