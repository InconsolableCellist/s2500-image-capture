#!/usr/bin/python3

from PIL import Image, ImageColor
import math

f = open('data.dat', 'rb')
byte = f.read(2)
count = 0
# im = Image.new('1', (4096,500))
im = Image.new('1', (4096,500))
color = ImageColor.getcolor('white', '1')
min = 99999
max = 0
while byte != b"" and count<4096:
    y = math.floor(int.from_bytes(byte, "little")/1024*500)
    if (y>=500):
        y = 499
    print("({}, {})".format(count, y))
    im.putpixel((count, y), color)
    if (y < min):
        min = y
    if (y > max):
        max = y
    # byte = f.read(20)
    byte = f.read(2)
    count += 1


print("min {} max {}".format(min, max))
im.show()