import time
import serial
import numpy as np
from.parsers.hipnuc_serial_parser import hipnuc_parser


class IMU:
    def __init__(self, port, baudrate):
        self.quat_data = None
        self.gvec_data = None
        self.ser = serial.Serial(port, int(baudrate), timeout=1)
        self.serial_parser = hipnuc_parser()

    def cmd_read(self, ):
        ser = self.ser
        total_start_time = time.time()

        try:
            # 打开串口
            # serial_open_start = time.time()
                # serial_open_end = time.time()
                # print(f"打开串口用时: {serial_open_end - serial_open_start:.6f} 秒")

                # 等待有数据可读
                # wait_data_start = time.time()
                # while not ser.in_waiting:
                #     time.sleep(0.001)
                # wait_data_end = time.time()
                # print(f"等待数据用时: {wait_data_end - wait_data_start:.6f} 秒")

                # 读取数据
                read_data_start = time.time()
                data = ser.read(ser.in_waiting)
                read_data_end = time.time()
                print(f"读取数据用时: {read_data_end - read_data_start:.6f} 秒")

                # 解析数据
                parse_data_start = time.time()
                hipnuc_frames = self.serial_parser.parse(data)
                parse_data_end = time.time()
                print(f"解析数据用时: {parse_data_end - parse_data_start:.6f} 秒")

                if hipnuc_frames:
                    latest_hipnuc_frame = hipnuc_frames[-1]
                    if latest_hipnuc_frame.frame_type is not None:
                        # 提取并转换四元数数据
                        extract_quat_start = time.time()
                        self.quat_data = np.array(latest_hipnuc_frame.quat, dtype=np.double)
                        extract_quat_end = time.time()
                        print(f"提取并转换四元数数据用时: {extract_quat_end - extract_quat_start:.6f} 秒")

                        # 提取并转换陀螺仪数据
                        extract_gvec_start = time.time()
                        self.gvec_data = np.array(latest_hipnuc_frame.gyr, dtype=np.double) * (np.pi / 180)
                        extract_gvec_end = time.time()
                        print(f"提取并转换陀螺仪数据用时: {extract_gvec_end - extract_gvec_start:.6f} 秒")

        except serial.SerialException as e:
            print(f"串口连接错误: {e}")
        except Exception as e:
            print(f"发生未知错误: {e}")

        total_end_time = time.time()
        print(f"整个函数用时: {total_end_time - total_start_time:.6f} 秒")

        return self.quat_data, self.gvec_data
