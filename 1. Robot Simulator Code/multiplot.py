import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

plt.style.use('ggplot')
fig, axes = plt.subplots(nrows=1, ncols=4, figsize=(16, 4), sharey=True)
#plt.tight_layout()

def plot_data(df, nodes, axes):
    ax1, ax2, ax3, ax4 = axes
    if nodes == 10:
        sns.kdeplot(df['Metric'], cumulative=True, legend=False, ax=ax1)
        ax1.set_ylabel('ECDF', fontsize = 14)
        ax1.set_title('10 Nodes')
    elif nodes == 20:
        sns.kdeplot(df['Metric'], cumulative=True, legend=False, ax=ax2)
        ax2.set_title('20 Nodes')
    elif nodes == 30:
        sns.kdeplot(df['Metric'], cumulative=True, legend=False, ax=ax3)
        ax3.set_title('30 Nodes')
    else:
        sns.kdeplot(df['Metric'], cumulative=True, legend=False, ax=ax4)
        ax4.set_title('40 Nodes')

df1 = pd.DataFrame({'Metric':np.random.randint(0, 15, 1000)})    
df2 = pd.DataFrame({'Metric':np.random.randint(0, 15, 1000)})    
df3 = pd.DataFrame({'Metric':np.random.randint(0, 15, 1000)})  

nodes = [10, 20, 30, 40]


for i in range(4):
    plot_data(df1, nodes[i], axes)
    plot_data(df2, nodes[i], axes)
    plot_data(df3, nodes[i], axes)
plt.show()