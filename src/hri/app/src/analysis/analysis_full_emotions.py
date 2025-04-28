import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# get all csv 
path ="..\data\\"
final_df = pd.DataFrame()
for root, dirs, files in sorted(os.walk(path)):
    for file in files:
        if(file.endswith("full.csv")):
            print(os.path.join(root,file))
            # one data frame for file
            df = pd.read_csv(os.path.join(root,file), sep=',')
            # concat dataframes
            final_df = pd.concat([final_df, df], ignore_index=True)

print("********************************\n \
       ********************************\n",
       final_df)

# 1 model confidence
plt.figure(figsize=(12, 6))
sns.boxplot(x='emotion', y='model_confidence', hue='condition', data=final_df)
plt.xlabel('Emotion')
plt.ylabel('Score (Accuracy)')
plt.title('Distribution of Emotion Scores by Condition')
plt.legend(title='Condition')
plt.savefig('emotion_confidence_full.png')

# 2 istogramma
plt.figure(figsize=(12, 6))
sns.countplot(x='emotion', hue='condition', data=final_df, palette='Set3')
plt.xlabel('Emotion')
plt.ylabel('Count')
plt.title('Distribution of Emotions by Condition')
plt.savefig('emotion_histogram_full.png')

# 3 box plot
emotions = final_df['emotion']
emotion_mapping = {
    "fear": -1,
    "angry": 0,
    "sad": 1,
    "disgust": 2,
    "neutral": 3,
    "happy": 4
}
final_df['emotion_numeric'] = final_df['emotion'].map(emotion_mapping)

plt.figure(figsize=(10, 6))
sns.boxplot(x='condition', y='emotion_numeric', data=final_df)
plt.xlabel('Condition')
plt.ylabel('Emotion')
plt.title('Boxplot of Emotions by Condition')
plt.xticks([0, 1], ['E-ToM', 'ToM'])
plt.yticks(list(emotion_mapping.values()), list(emotion_mapping.keys()))
plt.savefig('emotion_box_plot_full.png')