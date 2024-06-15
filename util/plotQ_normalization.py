import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# define robot's action
actions = ['no_help', 'sugg_rc', 'sugg_card']

# define states: {robot_last_action X game_state x user_state}
states = [
    'INIT_STATE',

    '(no_help, beg, f_correct)',
    '(no_help, beg, f_wrong, low_attempts)',
    '(no_help, beg, f_wrong, medium_attempts)',
    '(no_help, beg, f_wrong, high_attempts)',
    
    '(no_help, mid, f_correct)',
    '(no_help, mid, f_wrong, low_attempts)',
    '(no_help, mid, f_wrong, medium_attempts)',
    '(no_help, mid, f_wrong, high_attempts)',
    
    '(no_help, end, f_correct)',
    '(no_help, end, f_wrong, low_attempts)',
    '(no_help, end, f_wrong, medium_attempts)',
    '(no_help, end, f_wrong, high_attempts)',
    
    '(sugg_row, beg, f_correct)',
    '(sugg_row, beg, f_wrong, low_attempts)',
    '(sugg_row, beg, f_wrong, medium_attempts)',
    '(sugg_row, beg, f_wrong, high_attempts)',
    
    '(sugg_row, mid, f_correct)',
    '(sugg_row, mid, f_wrong, low_attempts)',
    '(sugg_row, mid, f_wrong, medium_attempts)',
    '(sugg_row, mid, f_wrong, high_attempts)',
    
    '(sugg_row, end, f_correct)',
    '(sugg_row, end, f_wrong, low_attempts)',
    '(sugg_row, end, f_wrong, medium_attempts)',
    '(sugg_row, end, f_wrong, high_attempts)',
    
    '(sugg_card, beg, f_correct)',
    '(sugg_card, beg, f_wrong, low_attempts)',
    '(sugg_card, beg, f_wrong, medium_attempts)',
    '(sugg_card, beg, f_wrong, high_attempts)',
    
    '(sugg_card, mid, f_correct)',
    '(sugg_card, mid, f_wrong, low_attempts)',
    '(sugg_card, mid, f_wrong, medium_attempts)',
    '(sugg_card, mid, f_wrong, high_attempts)',
    
    '(sugg_card, end, f_correct)',
    '(sugg_card, end, f_wrong, low_attempts)',
    '(sugg_card, end, f_wrong, medium_attempts)',
    '(sugg_card, end, f_wrong, high_attempts)',
    
    '(no_help, beg, s_correct)',
    '(no_help, beg, s_wrong, low_attempts)',
    '(no_help, beg, s_wrong, medium_attempts)',
    '(no_help, beg, s_wrong, high_attempts)',
    
    '(no_help, mid, s_correct)',
    '(no_help, mid, s_wrong, low_attempts)',
    '(no_help, mid, s_wrong, medium_attempts)',
    '(no_help, mid, s_wrong, high_attempts)',
    
    '(no_help, end, s_correct)',
    '(no_help, end, s_wrong, low_attempts)',
    '(no_help, end, s_wrong, medium_attempts)',
    '(no_help, end, s_wrong, high_attempts)',
    
    '(sugg_row, beg, s_correct)',
    '(sugg_row, beg, s_wrong, low_attempts)',
    '(sugg_row, beg, s_wrong, medium_attempts)',
    '(sugg_row, beg, s_wrong, high_attempts)',
    
    '(sugg_row, mid, s_correct)',
    '(sugg_row, mid, s_wrong, low_attempts)',
    '(sugg_row, mid, s_wrong, medium_attempts)',
    '(sugg_row, mid, s_wrong, high_attempts)',
    
    '(sugg_row, end, s_correct)',
    '(sugg_row, end, s_wrong, low_attempts)',
    '(sugg_row, end, s_wrong, medium_attempts)',
    '(sugg_row, end, s_wrong, high_attempts)',
    
    '(sugg_card, beg, s_correct)',
    '(sugg_card, beg, s_wrong, low_attempts)',
    '(sugg_card, beg, s_wrong, medium_attempts)',
    '(sugg_card, beg, s_wrong, high_attempts)',
    
    '(sugg_card, mid, s_correct)',
    '(sugg_card, mid, s_wrong, low_attempts)',
    '(sugg_card, mid, s_wrong, medium_attempts)',
    '(sugg_card, mid, s_wrong, high_attempts)',
    
    '(sugg_card, end, s_correct)',
    '(sugg_card, end, s_wrong, low_attempts)',
    '(sugg_card, end, s_wrong, medium_attempts)',
    '(sugg_card, end, s_wrong, high_attempts)',
]

# get q-table from file
Q = np.load('matrix.npy')

# swap axes
Q_transposed = np.transpose(Q)

# create the dataframe 
df = pd.DataFrame(Q_transposed, columns=states, index=actions)

# create a boolean mask of the columns that contain all zeros
mask = (df == 0).all()

# filter out the columns that match the mask
df_filtered = df.loc[:, ~mask]

df_filtered = (df_filtered - df_filtered.min()) / (df_filtered.max() - df_filtered.min())

sns.set(font_scale=0.9)
fig, ax = plt.subplots(figsize=(28, 9))

sns.heatmap(df_filtered, xticklabels=df_filtered.columns, yticklabels=actions, annot=True, fmt=".2f", cbar=True,
            linewidths=0.5, annot_kws={'fontsize':14, 'weight': 'bold'}, square=True,
            cbar_kws={'orientation': 'horizontal', 'pad': 0.05}
            )
ax.tick_params(axis="x", bottom=False, top=True, labelbottom=False, labeltop=True, labelrotation=90, labelsize=14)
ax.tick_params(axis="y", left=True, right=False, labelleft=True, labelright=False, labelsize=14)

cbar_ax = ax.collections[-1].colorbar.ax
cbar_ax.tick_params(labelsize=12)

plt.tight_layout()

plt.savefig('heatmap_normalized.pdf', format='pdf')