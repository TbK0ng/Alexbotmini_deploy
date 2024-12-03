# import time
# import serial
# import numpy as np
# from.parsers.hipnuc_serial_parser import hipnuc_parser


# class IMU:
#     def __init__(self):
#         self.quat_data = []  # 初始化类的属性quat为一个空列表，用于存放四元数数据
#         self.gvec_data = []  # 初始化类的属性gvec为一个空列表，用于存放陀螺仪数据（角速度向量）

#     def print_parsed_data(self, data):
#         """
#         Parse and store IMU data (both quaternion and gyroscope data) in the instance attributes.
#         """
#         if data.frame_type is not None:
#             # 逐个提取四元数数据，并转换为numpy.double类型，同时考虑单位转换（这里示例假设原始数据单位需转换为弧度，实际根据需求调整）
#             quat_data_0 = np.array(data.quat[0]).astype(np.double)
#             quat_data_1 = np.array(data.quat[1]).astype(np.double)
#             quat_data_2 = np.array(data.quat[2]).astype(np.double)
#             quat_data_3 = np.array(data.quat[3]).astype(np.double)
#             quat_data = np.array([quat_data_0, quat_data_1, quat_data_2, quat_data_3])
#             self.quat_data.append(quat_data)

#             # 逐个提取陀螺仪数据（角速度向量），转换为numpy.double类型，同时考虑单位转换（如从度/秒转换为弧度/秒，这里示例假设是这样的转换，按实际调整）
#             gvec_data_0 = np.array(data.gyr[0]).astype(np.double) * (np.pi / 180)
#             gvec_data_1 = np.array(data.gyr[1]).astype(np.double) * (np.pi / 180)
#             gvec_data_2 = np.array(data.gyr[2]).astype(np.double) * (np.pi / 180)
#             gvec_data = np.array([gvec_data_0, gvec_data_1, gvec_data_2])
#             self.gvec_data.append(gvec_data)

#     # def cmd_read(self, port, baudrate):
#     #     serial_parser = hipnuc_parser()
#     #     latest_hipnuc_frame = None
#     #     read_count = 0  # 用于记录已读取的次数
#     #     max_read_count = 1  # 设置最大读取次数为1次
#     #     quat_temp = []
#     #     gvec_temp = []

#     #     with serial.Serial(port, int(baudrate), timeout=1) as ser:
#     #         while read_count < max_read_count:
#     #             if ser.in_waiting:
#     #                 data = ser.read(ser.in_waiting)
#     #                 hipnuc_frames = serial_parser.parse(data)
#     #                 if hipnuc_frames:
#     #                     latest_hipnuc_frame = hipnuc_frames[-1]
#     #                     self.print_parsed_data(latest_hipnuc_frame)
#     #                     quat_temp.append(self.quat_data[-1])
#     #                     gvec_temp.append(self.gvec_data[-1])
#     #                     read_count += 1
#     #             time.sleep(0.01)

#     #     # # 合并临时数据列表和原数据列表，并转换为numpy.ndarray类型
#     #     # self.quat_data = np.array(quat_temp + (self.quat_data if self.quat_data else []))
#     #     # self.gvec_data = np.array(gvec_temp + (self.gvec_data if self.gvec_data else []))
#     #     return self.quat_data, self.gvec_data
    
#     def cmd_read(self, port, baudrate):
#         serial_parser = hipnuc_parser()
#         latest_hipnuc_frame = None
#         read_count = 0  # 用于记录已读取的次数
#         max_read_count = 1  # 设置最大读取次数为1次
#         quat_temp = []
#         gvec_temp = []

#         with serial.Serial(port, int(baudrate), timeout=1) as ser:
#             while read_count < max_read_count:
#                 if ser.in_waiting:
#                     data = ser.read(ser.in_waiting)
#                     hipnuc_frames = serial_parser.parse(data)
#                     if hipnuc_frames:
#                         latest_hipnuc_frame = hipnuc_frames[-1]
#                         self.print_parsed_data(latest_hipnuc_frame)
#                         quat_temp.append(self.quat_data[-1])
#                         gvec_temp.append(self.gvec_data[-1])
#                         read_count += 1
#                 time.sleep(0.01)

#         # 扁平化处理quat_temp列表，将其中的numpy数组元素合并为一个二维numpy数组，然后取第一行（假设每次只读取一组有效的四元数数据）
#         if quat_temp:
#             self.quat_data = np.vstack(quat_temp)[0]
#         else:
#             self.quat_data = np.array([])

#         # 扁平化处理gvec_temp列表，将其中的numpy数组元素合并为一个二维numpy数组，然后取第一行（假设每次只读取一组有效的陀螺仪数据）
#         if gvec_temp:
#             self.gvec_data = np.vstack(gvec_temp)[0]
#         else:
#             self.gvec_data = np.array([])

#         return self.quat_data, self.gvec_data

# if __name__ == "__main__":
#     # If there are multiple USB devices here, 
#     # replace them with the actual serial port names.
#     port = "/dev/ttyUSB0"
#     baudrate = 115200
#     imu = IMU()
#     quat_data, gvec_data = imu.cmd_read(port, baudrate)
#     # 这里可以根据后续需求进一步处理获取到的quat_data和gvec_data
#     print("Quaternion array:", quat_data[-10:])  # 打印quaternion数组的后12个元素（注意这里用的是返回的quat_data，就是类属性self.quat_data了）
#     print("Gyroscope array:", gvec_data[-10:])  # 打印gyroscope数组的后12个元素（同理用的是返回的gvec_data，即类属性self.gvec_data）
################################################################################################################################################
# import time
# import serial
# import numpy as np
# from.parsers.hipnuc_serial_parser import hipnuc_parser

# class IMU:
#     def __init__(self, data_storage_size):
#         self.quat_data = np.zeros((data_storage_size, 4))  # 初始化用于存放四元数数据的二维数组，根据传入参数指定行数，4列（四元数有4个元素）
#         self.gvec_data = np.zeros((data_storage_size, 3))  # 初始化用于存放陀螺仪数据（角速度向量，3个元素）的二维数组，根据传入参数指定行数，3列
#         self.data_index = 0  # 新增一个属性，用于记录当前数据存储的索引位置

# # 在使用IMU类实例化对象时，传入合适的N值
# N = 10  # 这里同样可以按照前面提到的定义固定值或者动态获取值的方式来确定N
# imu = IMU(N)


# def print_parsed_data(self, data):
#     """
#     Parse and store IMU data (both quaternion and gyroscope data) in the instance attributes.
#     """
#     if data.frame_type is not None:
#         # 逐个提取四元数数据，并转换为numpy.double类型，同时考虑单位转换（这里示例假设原始数据单位需转换为弧度，实际根据需求调整）
#         quat_data_0 = np.array(data.quat[0]).astype(np.double)
#         quat_data_1 = np.array(data.quat[1]).astype(np.double)
#         quat_data_2 = np.array(data.quat[2]).astype(np.double)
#         quat_data_3 = np.array(data.quat[3]).astype(np.double)
#         quat_data = np.array([quat_data_0, quat_data_1, quat_data_2, quat_data_3])

#         # 通过索引更新self.quat_data数组的对应元素
#         self.quat_data[self.data_index] = quat_data

#         # 逐个提取陀螺仪数据（角速度向量），转换为numpy.double类型，同时考虑单位转换（如从度/秒转换为弧度/秒，这里示例假设是这样的转换，按实际调整）
#         gvec_data_0 = np.array(data.gyr[0]).astype(np.double) * (np.pi / 180)
#         gvec_data_1 = np.array(data.gyr[1]).astype(np.double) * (np.pi / 180)
#         gvec_data_2 = np.array(data.gyr[2]).astype(np.double) * (np.pi / 180)
#         gvec_data = np.array([gvec_data_0, gvec_data_1, gvec_data_2])

#         # 通过索引更新self.gvec_data数组的对应元素
#         self.gvec_data[self.data_index] = gvec_data

#         self.data_index += 1  # 更新索引位置，指向下一个要存储数据的位置
#         if self.data_index >= self.quat_data.shape[0]:  # 判断是否超出了预设的存储范围
#             # 这里可以添加逻辑，比如重新分配更大的数组空间，或者覆盖旧数据等，根据实际需求决定
#             print("Data index has reached the maximum limit. Consider handling data differently.")

# def cmd_read(self, port, baudrate):
#     serial_parser = hipnuc_parser()
#     latest_hipnuc_frame = None
#     read_count = 0  # 用于记录已读取的次数
#     max_read_count = 1  # 设置最大读取次数为1次

#     with serial.Serial(port, int(baudrate), timeout=1) as ser:
#         while read_count < max_read_count:
#             if ser.in_waiting:
#                 data = ser.read(ser.in_waiting)
#                 hipnuc_frames = serial_parser.parse(data)
#                 if hipnuc_frames:
#                     latest_hipnuc_frame = hipnuc_frames[-1]
#                     self.print_parsed_data(latest_hipnuc_frame)
#                     read_count += 1
#             time.sleep(0.01)

#     return self.quat_data[self.data_index - 1], self.gvec_data[self.data_index - 1]  # 返回最后一次存储的数据
# if __name__ == "__main__":
#     # If there are multiple USB devices here, 
#     # replace them with the actual serial port names.
#     port = "/dev/ttyUSB0"
#     baudrate = 115200
#     imu = IMU()
#     quat_data, gvec_data = imu.cmd_read(port, baudrate)
#     # 这里可以根据后续需求进一步处理获取到的quat_data和gvec_data
#     print("Quaternion array:", quat_data[-10:])  # 打印quaternion数组的后12个元素（注意这里用的是返回的quat_data，就是类属性self.quat_data了）
#     print("Gyroscope array:", gvec_data[-10:])  # 打印gyroscope数组的后12个元素（同理用的是返回的gvec_data，即类属性self.gvec_data）
#######################################################################################################################################################

import time
import serial
import numpy as np
from.parsers.hipnuc_serial_parser import hipnuc_parser
import time
import serial
import numpy as np
from.parsers.hipnuc_serial_parser import hipnuc_parser


class IMU:
    def __init__(self):
        self.quat_data = None
        self.gvec_data = None

    def cmd_read(self, port, baudrate):
        serial_parser = hipnuc_parser()
        latest_hipnuc_frame = None
        read_count = 0  # 记录已读取的次数
        max_read_count = 1  # 设置最大读取次数

        with serial.Serial(port, int(baudrate), timeout=1) as ser:
            while read_count < max_read_count:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    hipnuc_frames = serial_parser.parse(data)
                    if hipnuc_frames:
                        latest_hipnuc_frame = hipnuc_frames[-1]
                        if latest_hipnuc_frame.frame_type is not None:
                            # 提取并转换四元数数据
                            quat_data_0 = np.array(latest_hipnuc_frame.quat[0]).astype(np.double)
                            quat_data_1 = np.array(latest_hipnuc_frame.quat[1]).astype(np.double)
                            quat_data_2 = np.array(latest_hipnuc_frame.quat[2]).astype(np.double)
                            quat_data_3 = np.array(latest_hipnuc_frame.quat[3]).astype(np.double)
                            self.quat_data = np.array([quat_data_0, quat_data_1, quat_data_2, quat_data_3])

                            # 提取并转换陀螺仪数据
                            gvec_data_0 = np.array(latest_hipnuc_frame.gyr[0]).astype(np.double) * (np.pi / 180)
                            gvec_data_1 = np.array(latest_hipnuc_frame.gyr[1]).astype(np.double) * (np.pi / 180)
                            gvec_data_2 = np.array(latest_hipnuc_frame.gyr[2]).astype(np.double) * (np.pi / 180)
                            self.gvec_data = np.array([gvec_data_0, gvec_data_1, gvec_data_2])
                        read_count += 1
                time.sleep(0.01)

        return self.quat_data, self.gvec_data
