import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
 
file_path = "/home/y-takahashi/catkin_ws/src/nav_cloning/data/loss/00_02/com.csv"

file_names = [file_path]
df = pd.read_csv(file_path, )

sns.set(font_scale = 2.3)

fig, ax = plt.subplots(figsize = (12,8))
ax.plot()
ax.set(xlabel ='Step', ylabel='Loss', xlim=(0,4000), ylim=(0,0.05))

sns.set_style("darkgrid")
# plt.figure(figsize = (12, 8))
sns.lineplot(x="x", y="y", data=df, ci="sd")
# plt.xlabel("Step")
# plt.ylabel("Loss")
plt.legend()
plt.show()
