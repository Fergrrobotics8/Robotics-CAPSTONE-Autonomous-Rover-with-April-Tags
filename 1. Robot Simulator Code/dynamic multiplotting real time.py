import matplotlib.pyplot as plt
import numpy as np
from math import cos

# Some example data to display
x, y = [], []
i=0
fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(16, 8))

def plotgod(x,y):
    ax1, ax2, ax3 = axes
    
    
    ax1.set_title('X position')
    ax2.set_title('Y position')
    ax3.set_title('Angle theta')

    ax3.plot(x,y, color = "b")
    ax1.plot(x, y, color ="r")
    ax2.plot(x, y, color = "g")

    plt.pause(0.001)


while i<20:
    x.append(i)
    y.append(cos(i))
    plotgod(x,y)
    i+=0.5


plt.show()