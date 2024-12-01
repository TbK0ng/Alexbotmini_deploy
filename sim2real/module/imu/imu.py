import time
import serial
import numpy as np
from parsers.hipnuc_serial_parser import hipnuc_parser
import time as t

class IMU:
    def __init__(self):
        self.quat = []  
        self.gvec = []  

    def print_parsed_data(self, data):
        """
        Parse and store IMU data (both quaternion and gyroscope data) in the instance attributes.
        Also print the parsed IMU data in a readable format.
        """
        if data.frame_type is not None:
            quat_data = np.array([data.quat[0], data.quat[1], data.quat[2], data.quat[3]]).astype(np.double)
            self.quat.append(quat_data)
            gvec_data = np.array([data.gyr[0], data.gyr[1], data.gyr[2]]).astype(np.double) * (np.pi / 180)
            self.gvec.append(gvec_data)
            # 打印四元数数据数组，格式化输出每个元素
            print("Quaternion: ", [ "%.6f" % element for element in quat_data ])
            # 打印陀螺仪数据（角速度向量）数组，格式化输出每个元素
            print("Gyroscope (rad/s): ", [ "%.6f" % element for element in gvec_data ])

    def cmd_read(self, port, baudrate):
        serial_parser = hipnuc_parser()
        latest_hipnuc_frame = None
        while True:
            start_time = t.perf_counter()
            start_time = t.perf_counter()
            with serial.Serial(port, int(baudrate), timeout=1) as ser:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    hipnuc_frames = serial_parser.parse(data)
                    if hipnuc_frames:
                        latest_hipnuc_frame = hipnuc_frames[-1]
                        self.print_parsed_data(latest_hipnuc_frame)
            end_time = t.perf_counter()
            time.sleep(0.00889 - end_time + start_time)
            end_time = t.perf_counter()
            # print("接收并处理一帧数据的频率: %.2f Hz" % (1 / (end_time - start_time)))
            # 打印四元数数组self.quat中的数据，逐行打印每个元素列表
            print("Quaternion Array:")
            for quat_element in self.quat:
                print(quat_element)  # 简化为直接打印元素本身
            # 打印陀螺仪数据数组self.gvec中的数据，逐行打印每个元素列表
            print("Gyroscope Array:")
            for gvec_element in self.gvec:
                print(gvec_element)  # 简化为直接打印元素本身

            # 防止cpu死机
            time.sleep(0.001)



if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    baudrate = 115200
    imu = IMU()
    result = imu.cmd_read(port, baudrate)
