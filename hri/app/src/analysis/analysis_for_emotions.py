import pandas as pd
import matplotlib.pyplot as plt

# Carica i dati dal file CSV
df = pd.read_csv('../data/emotion_data.csv')

# Estrai solo la colonna dell'emozione
emotions = df['emotion']

# Calcola il conteggio di ciascuna emozione
emotion_counts = emotions.value_counts()

# Crea il box plot basato sul conteggio delle emozioni
plt.figure(figsize=(8, 6))
plt.boxplot(emotion_counts.values, patch_artist=True, showmeans=True)

# Impostazioni del plot
plt.title('Box Plot per Emozioni')
plt.xlabel('Emozione')
plt.ylabel('Conteggio')
plt.xticks(range(1, len(emotion_counts.index) + 1), emotion_counts.index)  # Etichette sull'asse x
plt.grid(True)

# Mostra il plot
plt.tight_layout()
plt.show()
