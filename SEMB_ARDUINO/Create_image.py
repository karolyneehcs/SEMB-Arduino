# -*- coding: utf-8 -*-
import sys
f = open("OUTPUT.pgm",'wb')
image_array = sys.argv

image_array.pop(0)

string = image_array.pop(0) + '\n'
f.write(bytes(string,'utf-8'))

string = image_array.pop(0) + ' '
f.write(bytes(string,'utf-8'))

string = image_array.pop(0) + '\n'
f.write(bytes(string,'utf-8'))

string = image_array.pop(0) + '\n'
f.write(bytes(string,'utf-8'))

for element in image_array:
    f.write(bytes([int(element)]))

f.close()