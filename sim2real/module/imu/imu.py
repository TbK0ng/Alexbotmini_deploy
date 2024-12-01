
import time
import serial
from .parsers.hipnuc_serial_parser import hipnuc_parser
# sudo chmod a+rw /dev/ttyUSB0
class imu:
    def print_parsed_data(data):
        """Format and print IMU data in a professional and compact format"""
        if data.frame_type is not None:
            data_fields = [
                ("Gyroscope (deg/s)",f"({data.gyr[0]:<9.3f}, {data.gyr[1]:<9.3f}, {data.gyr[2]:<9.3f})" if data.gyr is not None else None),
                ("Quaternion",       f"({data.quat[0]:<9.3f}, {data.quat[1]:<9.3f}, {data.quat[2]:<9.3f}, {data.quat[3]:<9.3f})" if data.quat is not None else None),
            ]
            # Print the data fields
            for label, value in data_fields:
                if value is not None:
                    print(f"{label:<24}: {value}")

    def cmd_read(port, baudrate):
        serial_parser = hipnuc_parser()
        latest_hipnuc_frame = None

        with serial.Serial(port, int(baudrate), timeout=1) as ser:
            while True:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)

                    hipnuc_frames = serial_parser.parse(data)

                    if hipnuc_frames:
                        latest_hipnuc_frame = hipnuc_frames[-1]
                        imu.print_parsed_data(latest_hipnuc_frame)  
                time.sleep(0.001)  # Small delay to prevent CPU overuse


if __name__ == "__main__":
    # If there are multiple USB devices here, 
    # replace them with the actual serial port names.
    port = "/dev/ttyUSB0"  
    baudrate = 115200
    imu = imu.cmd_read(port, baudrate)

