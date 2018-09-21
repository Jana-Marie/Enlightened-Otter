#!/usr/bin/python3.7
import math

gamma       = 1.6
outputRange = 500
steps	    = 500

print("float gammaTable[] = { // generated gamma correction table from Tools/gamma.py gamma value used: %.1f resolution: %.5f " % (gamma,outputRange/steps),end='')

for i in range (0,steps,1):

  if(i > 0):
    print(",",end='')
  if((i & 15) == 0):
    print()
  print("\t%.2f" % round((math.pow(i / steps, gamma) * steps + 0.5)/steps*outputRange,2),end='')

print(" };")
