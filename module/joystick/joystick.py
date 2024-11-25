import os, struct, array
from fcntl import ioctl

print('Welcome to alexbot joystick_xbox')

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print('  /dev/input/%s' % (fn))
# 这句显示手柄在硬件中的端口位置： /dev/input/js0
# We'll store the states here.
axis_states = {}
button_states = {}

# 先前校验时，方向盘是x,左侧踏板是z,右侧踏板是rz。


# These constants were borrowed from linux/input.h
axis_names = {
    0x00: 'x',
    0x01: 'y',#y获取y轴的值----------------    
}

axis_map = []

# Open the joystick device.打开操作杆装置
fn = '/dev/input/js0'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')#以二进制读模式打开

# # Get the device name.
buf = array.array('u', ['\0'] * 5)
#是设备驱动程序中对I/O设备管理的函数。例如：串口的传输波特率和马达的转速
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
js_name = buf.tostring()#装置名字
print('Device name: %s' % js_name)

# Get number of axes and buttons.轴
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf)  # JSIOCGAXES
num_axes = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf)  # JSIOCGAXMAP
#
for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

while True:
    evbuf = jsdev.read(8)
    if evbuf:
        time, value, type, number = struct.unpack('IhBB', evbuf)

        if type & 0x02:
            axis = axis_map[number]
            if axis:
                # print("{}".format(axis))
                if axis == "x":
                    fvalue = value / 32767

                    axis_states[axis] = fvalue
                    print("%s: %.3f" % (axis, fvalue))
                elif axis == "y":
                    fvalue = value / 32767

                    axis_states[axis] = fvalue
                    print("%s: %.3f" % (axis, fvalue))
        elif type & 0x01:
            if number==0x00:#0号按钮
                print(number,'=',value)
            elif number==0x01:
                print(number,'=',value)
            elif number==0x02:
                print(number,'=',value)
            elif number==0x03:
                print(number,'=',value)
            elif number==0x04:
                print(number,'=',value)
            elif number == 0x05:
                print(number, '=', value)
            elif number == 0x06:
                print(number, '=', value)
