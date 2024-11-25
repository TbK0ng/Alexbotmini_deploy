import os
import struct
import array
from fcntl import ioctl

print('Welcome to alexbot joystick_xbox')

axis_names = {
    0x00: 'abs_x',  # 左摇杆X轴
    0x01: 'abs_y',  # 左摇杆Y轴
    0x02: 'lt',  # 右摇杆X轴
    0x03: 'abs_ry',  # 右摇杆Y轴
    0x04: 'abs_rx',  # 左扳机键（类比L2）
    0x05: 'rt',  # 右扳机键（类比R2）
    0x10: 'dpad_x',  # 十字键X轴
    0x11: 'dpad_y',  # 十字键Y轴
}

# Xbox手柄按钮名称映射字典，同样根据实际情况调整
button_names = {
    0x130: 'button_a',  # A按钮
    0x131: 'button_b',  # B按钮
    0x132: 'button_a',  # X按钮
    0x133: 'button_x',  # Y按钮
    0x134: 'button_y',  # LB按钮（类比L1）
    0x135: 'button_b',  # RB按钮（类比R1）
    0x136: 'button_lb',  # Back按钮（类比Options）
    0x137: 'button_rb',  # Start按钮（类比Create）
    0x138: 'button_guide',  # Guide按钮（类比PS按钮）
    0x139: 'button_xbox',  # Xbox按钮
}

axis_map = []
button_map = []
# 打开PS5手柄设备
fn = '/dev/input/js0'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')
def xbox_open(device_path='/dev/input/js0'):
    fn = '/dev/input/js0'
    print('Opening %s...' % fn)
    jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('B', [0] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
print('Device name: %s' % js_name)
 
# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]
 
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]
 
# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP
 
for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
 
# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP
 
for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
 
print('%d axes found: %s' % (num_axes, ', '.join(axis_map)))
print('%d buttons found: %s' % (num_buttons, ', '.join(button_map)))

# Main event loop
while True:
    evbuf = jsdev.read(8)
    if evbuf:
        time, value, type, number = struct.unpack('IhBB', evbuf) #图中标出的数字是指此处的 number，用来判断此词数据是哪个按键的变化
        if type & 0x01:
            button = button_map[number]
            if button:
                button_names[button] = value
                if value:
                    print("%s pressed" % (button))
                else:
                    print("%s released" % (button))
 
        if type & 0x02:
            axis = axis_map[number]
            if number==0x01 :
                value=-value
            if number==0x05 :
                value=-value
            
            if value > 5000:
                fvalue = value / 32767.0
                axis_names[axis] = fvalue
                print("%s: %.3f" % (axis, fvalue))
            if value < -5000:
                fvalue = value / 32767.0
                axis_names[axis] = fvalue
                print("%s: %.3f" % (axis, fvalue))