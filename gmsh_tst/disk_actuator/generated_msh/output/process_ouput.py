import matplotlib.pyplot as plt
import numpy as np
import os,sys
import matplotlib.pylab as pl


files = os.listdir(".")
files.sort()
files.reverse()
lines = []
for f in files:
    if f[-3:] != "npy":
        continue
    print(f)
    x = np.load(f)
    print(x[0])
    lines.append(x)

lines = np.array(lines)
colors =  pl.cm.jet(np.linspace(0, 1, len(lines)))


pin = 601

for i,x in enumerate(lines):
   plt.plot(x[2:,0,2], color=colors[i])
plt.show()



data = []
length_scales = []
for x in lines:
    data.append(x[pin,1, 2])
    length_scales.append(x[0,0,0])

plt.scatter(length_scales, data)
plt.xlim(1,0)
plt.title("Velocity and lengthscale on at time " + str(pin*x[0,0,2]))
plt.xlabel("Mesh Length Scale")
plt.ylabel("velocity")
plt.show()
