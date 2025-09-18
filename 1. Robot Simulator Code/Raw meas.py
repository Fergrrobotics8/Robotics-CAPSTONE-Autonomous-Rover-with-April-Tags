import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

plt.style.use('ggplot')
fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(16, 8), sharey=True)
#plt.tight_layout()

def plot_data(x, y, axes):
    ax1, ax2, ax3 = axes

    
    ax1.set_title('X position')
    ax2.set_title('Y position')
    ax3.set_title('Angle theta')


meas=[0.759,  0.189, -0.875]
x=[1,2,3]

plot_data(x, meas[0], axes)
plot_data(x, meas[1], axes)
plot_data(x, meas[2], axes)

plt.show()