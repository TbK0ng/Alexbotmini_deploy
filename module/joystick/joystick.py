import os
import struct
import array
from fcntl import ioctl

print('Welcome to alexbot joystick_xbox')

axis_names = {
    0x00: 'abs_lx',  # 左摇杆X轴
    0x01: 'abs_ly',  # 左摇杆Y轴
    0x02: 'lt',  # 左扳机键
    0x03: 'abs_rx',  # 右摇杆x轴
    0x04: 'abs_ry',  # 右摇杆y轴
    0x05: 'rt',  # 右扳机键
    0x10: 'cross_x',  # 十字键X轴
    0x11: 'cross_y',  # 十字键Y轴
}

# Xbox手柄按钮名称映射字典，同样根据实际情况调整
button_names = {
    0x130: 'button_a',  # A按钮
    0x131: 'button_b',  # B按钮
    0x132: 'button_2',
    0x133: 'button_x',  # X按钮
    0x134: 'button_y',  # Y按钮
    0x135: 'button_1',
    0x136: 'button_lb',  # LB按钮（类比L1）
    0x137: 'button_rb',  # RB按钮（类比R1）
    0x13a: 'button_-',  # Start按钮（类比Create）
    0x13b: 'button_+',  # Guide按钮（类比PS按钮）
    0x13c: 'button_xbox',  # Xbox按钮
}

axis_map = []
button_map = []

# 打开XBOX手柄设备
fn = '/dev/input/js0'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')


# 获取设备名称
buf = array.array('B', [0] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
print('Device name: %s' % js_name)

# 获取轴数量和按钮数量
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf)  # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
num_buttons = buf[0]

# 获取轴映射
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf)  # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)

# 获取按钮映射
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)

print('%d axes found: %s' % (num_axes, ', '.join(axis_map)))
print('%d buttons found: %s' % (num_buttons, ', '.join(button_map)))

# 主事件循环
while True:
    evbuf = jsdev.read(8)
    if evbuf:
        time, value, type, number = struct.unpack('IhBB', evbuf)
        if type & 0x01:  # 判断是否是按钮事件
            button = button_map[number]
            if button:
                button_names[button] = value
                if value:
                    print("%s pressed" % (button))
                else:
                    print("%s released" % (button))
        elif type & 0x02:  # 判断是否是轴事件
            axis = axis_map[number]
            # 针对cross_y（0x11）轴进行值反转处理，放在轴事件分支中
            if number == 0x11:
                value = -value
            if number == 0x01:
                value = -value
            if number == 0x04:
                value = -value
            if number == 0x05:
                value = -value
            if number == 0x02:
                value = -value

            if value > 5000 or value < -5000:
                fvalue = value / 32767.0
                axis_names[axis] = fvalue
                print("%s: %.3f" % (axis, fvalue))
