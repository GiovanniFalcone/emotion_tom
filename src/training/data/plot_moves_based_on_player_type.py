import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

with open('game_perfect.npy', 'rb') as f:
    array1 = np.load(f)

with open('game_imperfect.npy', 'rb') as f:
    array2 = np.load(f)

with open('game_helped.npy', 'rb') as f:
    array3 = np.load(f)

# Get mean and std for each player type
mean1, std1 = np.mean(array1), np.std(array1)
mean2, std2 = np.mean(array2), np.std(array2)
mean3, std3 = np.mean(array3), np.std(array3)

print("****** Game length *****",
      "\nmean and std for perfect player:", mean1, std1,
      "\nmean and std for imperfect player:", mean2, std2,
      "\nmean and std for helped player:", mean3, std3)

# DataFrame creation
data = pd.DataFrame({'perfect': array1, 'avg': array2, 'helped': array3}, index=range(len(array1)))

# Plot with seaborn using violin plot
fig, ax = plt.subplots()
sns.violinplot(data=data)

# Setting label
plt.xticks(range(len(data.columns)), data.columns)

# Save the plot
fig.savefig("violin.pdf")
plt.close(fig)