# -*- coding: utf-8 -*-
import sys
path = sys.argv[1]
f = open(path,'rb')

g = f.read()
#b = bytearray(g,'utf-8')

for x in g:
    print("%d," %x,end='')

print("'",end='')
print("\\0",end='')
print("'",end='')
