import pandas as pd
import scipy.stats as stats 

from scipy.stats import shapiro
from statsmodels.stats.multicomp import pairwise_tukeyhsd

# leggi il file e salva i dati in df
df = pd.read_excel("results.xlsx")

# Converto in secondi il tempo
df['time to finish'] = pd.to_timedelta(df['time to finish'])
df['time to finish'] = df['time to finish'].dt.total_seconds()
df['average time to find a pair'] = pd.to_timedelta(df['average time to find a pair'])
df['average time to find a pair'] = df['average time to find a pair'].dt.total_seconds()

# filtra le modalità 
superficial = df[df["robot_type"] == 'superficial_deception']
external = df[df["robot_type"] == 'external_deception']
hidden = df[df["robot_type"] == 'hidden_deception']

# colonne da analizzare
variables = ['turns', 'time to finish', 'average time to find a pair', 'number of suggestions',
             'number of card suggestions', 'number of row/col suggestions', 'number of suggestions for first card',
             'number of suggestions followed', 'number of card suggestions not followed',
             'number of row/col suggestions not followed', 'number of suggestions for first card not followed',
             'suggestions for the first card that let to a match']

kruskal_results = {'Variable': [], 
                 'Mean Superficial': [], 'Std Superficial': [],
                 'Mean External': [], 'Std External': [],
                 'Mean Hidden': [], 'Std Hidden': [],
                 'F-Statistic': [], 'P-Value': []}

for variable in variables:
    # stampa risultati per assunzioni anova + test non parametrico
    print(variable, "\n\t", shapiro(superficial[variable]), "\n\t", shapiro(external[variable]), "\n\t", shapiro(hidden[variable]), # normale
                    "\n\t", stats.bartlett(superficial[variable], external[variable], hidden[variable]),                            # var uguali
                    "\n\t", stats.kruskal(superficial[variable], external[variable], hidden[variable]), "\n")                       # non parametrico
    # per test non parametrico
    f_statistic, p_value = stats.kruskal(superficial[variable], external[variable], hidden[variable])
    kruskal_results['Variable'].append(variable)
    kruskal_results['Mean Superficial'].append(superficial[variable].mean())
    kruskal_results['Std Superficial'].append(superficial[variable].std())
    kruskal_results['Mean External'].append(external[variable].mean())
    kruskal_results['Std External'].append(external[variable].std())
    kruskal_results['Mean Hidden'].append(hidden[variable].mean())
    kruskal_results['Std Hidden'].append(hidden[variable].std())
    kruskal_results['F-Statistic'].append(f_statistic)
    kruskal_results['P-Value'].append(p_value)

# per leggibilità arrotondo tutto tranne P-value
kruskal_df = pd.DataFrame(kruskal_results)
rounded_columns = [col for col in kruskal_df.columns if col != 'P-Value']
new_df = kruskal_df.copy()
new_df[rounded_columns] = new_df[rounded_columns].round(4)

# post-hoc test
data = pd.concat([superficial, external, hidden])
tukey_results_list = []
for variable in variables:
    tukey_result = pairwise_tukeyhsd(data[variable], data['robot_type'])
    # crea il dataframe
    tukey_result_df = pd.DataFrame(data=tukey_result._results_table.data[1:], columns=tukey_result._results_table.data[0])
    tukey_result_df['Variable'] = variable
    # aggiungi il dataframe alla lista
    tukey_results_list.append(tukey_result_df)

# concatena i risultati
tukey_df = pd.concat(tukey_results_list, ignore_index=True)

# scrivo su file i risultati
with pd.ExcelWriter('Test_general_results.xlsx') as writer:  
    new_df.to_excel(writer, sheet_name='Kruskal_results')
    tukey_df.to_excel(writer, sheet_name='Tukey_results')