import time
import serial
import numpy as np
from parsers.hipnuc_serial_parser import hipnuc_parser

# sudo chmod a+rw /dev/ttyUSB0

class IMU:
    def __init__(self):
        self.quat = []  # 初始化类的属性quat为一个空列表，用于存放四元数数据
        self.gvec = []  # 初始化类的属性gvec为一个空列表，用于存放陀螺仪数据（角速度向量）

    def print_parsed_data(self, data):
        """
        Parse and store IMU data (both quaternion and gyroscope data) in the instance attributes.
        Also print the parsed IMU data in a readable format.
        """
        if data.frame_type is not None:
            # 正确提取四元数数据，并转换为numpy.double类型，同时考虑单位转换（这里示例假设原始数据单位需转换为弧度，实际根据需求调整）
            quat_data = np.array([data.quat[0], data.quat[1], data.quat[2], data.quat[3]]).astype(np.double)
            self.quat.append(quat_data)

            # 正确提取陀螺仪数据（角速度向量），转换为numpy.double类型，同时考虑单位转换（如从度/秒转换为弧度/秒，这里示例假设是这样的转换，按实际调整）
            gvec_data = np.array([data.gyr[0], data.gyr[1], data.gyr[2]]).astype(np.double) * (np.pi / 180)
            self.gvec.append(gvec_data)

            # # 打印四元数数据，使用格式化字符串让输出更规整
            # print("Quaternion: [%.6f, %.6f, %.6f, %.6f]" % (quat_data[0], quat_data[1], quat_data[2], quat_data[3]))
            # # 打印陀螺仪数据（角速度向量），同样使用格式化字符串让输出更规整
            # print("Gyroscope (rad/s): [%.6f, %.6f, %.6f]" % (gvec_data[0], gvec_data[1], gvec_data[2]))
            print("Quaternion array:", self.quat[-1:])  # 打印quaternion数组的后12个元素
            print("Gyroscope array:", self.gvec[-1:])  # 打印gyroscope数组的后12个元素

    def cmd_read(self, port, baudrate):
        serial_parser = hipnuc_parser()
        latest_hipnuc_frame = None
        read_count = 0  # 用于记录已读取的次数
        max_read_count = 1  # 设置最大读取次数为1次

        with serial.Serial(port, int(baudrate), timeout=1) as ser:
            while read_count < max_read_count:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    hipnuc_frames = serial_parser.parse(data)
                    if hipnuc_frames:
                        latest_hipnuc_frame = hipnuc_frames[-1]
                        self.print_parsed_data(latest_hipnuc_frame)
                        read_count += 1  
                # 控制循环频率，想要达到大约100Hz的频率，每秒100次循环，每次循环间隔大约就是1/100秒，即0.01秒
                time.sleep(0.01)
                # 要imu先读取再写入数组，所以顺序有问题



if __name__ == "__main__":
    # If there are multiple USB devices here, 
    # replace them with the actual serial port names.
    port = "/dev/ttyUSB0"
    baudrate = 115200
    imu = IMU()
    result = imu.cmd_read(port, baudrate)
    print("Quaternion array:", imu.quat[-1:])  # 打印quaternion数组的后12个元素
    print("Gyroscope array:", imu.gvec[-1:])  # 打印gyroscope数组的后12个元素
