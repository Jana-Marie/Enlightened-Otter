#!/usr/bin/python3.7
import math

gamma       = 2.8
outputRange = 1
steps       = 1000

print("float gamma[] = {",end='')

for i in range (0,steps,outputRange):

  if(i > 0):
    print(",",end='')
  if((i & 15) == 0):
    print()
  print("\t%.4f" % round((math.pow(i / steps, gamma) * steps + 0.5)/(steps*outputRange),4),end='')

print(" };")
exit()