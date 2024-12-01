import time
import serial
from .parsers.hipnuc_serial_parser import hipnuc_parser
# sudo chmod a+rw /dev/ttyUSB0
class imu:
    
    def read_imu_data(port, baudrate):
        """
        从指定串口读取IMU数据，提取四元数和角速度数据并返回
        """
        serial_parser = hipnuc_parser()
        quaternion_data = []
        gyroscope_data = []

        with serial.Serial(port, int(baudrate), timeout=1) as ser:
            while True:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)

                    hipnuc_frames = serial_parser.parse(data)

                    if hipnuc_frames:
                        latest_hipnuc_frame = hipnuc_frames[-1]
                        if latest_hipnuc_frame.gyr is not None:
                            gyroscope_data.append((latest_hipnuc_frame.gyr[0], latest_hipnuc_frame.gyr[1], latest_hipnuc_frame.gyr[2]))
                        if latest_hipnuc_frame.quat is not None:
                            quaternion_data.append((latest_hipnuc_frame.quat[0], latest_hipnuc_frame.quat[1], latest_hipnuc_frame.quat[2], latest_hipnuc_frame.quat[3]))

                time.sleep(0.001)  # Small delay to prevent CPU overuse

        return quaternion_data, gyroscope_data

if __name__ == "__main__":
    # 如果这里有多USB设备，替换为实际的串口名称
    port = "/dev/ttyUSB0"
    baudrate = 115200
    quaternion_data, gyroscope_data = imu.read_imu_data(port, baudrate)
    print("四元数数据:", quaternion_data)
    print("角速度数据:", gyroscope_data)
    
