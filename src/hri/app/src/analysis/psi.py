import sys
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import scipy.stats as stats 
from scipy.stats import f_oneway, shapiro, tukey_hsd
from statsmodels.stats.multicomp import pairwise_tukeyhsd

def process_data(type_filter):
    df = pd.read_excel("deception_responses.xlsx")
    # solo le righe della modalità desiderata
    df = df[df["Type"] == type_filter]
    # solo le colonne PSI
    df = df.iloc[:, 6:22]
    # reverse SOC
    df["Questo robot: [non ha capacità sociali]"] = 6 - df["Questo robot: [non ha capacità sociali]"]
    # reverse AC
    df["Questo robot: [ignora ciò che le persone pensano]"] = 6 - df["Questo robot: [ignora ciò che le persone pensano]"]
    # prendo le colonne SOC, AC, ...
    colonne_da_sommare = [(0, 4), (4, 8), (8, 12), (12, 16)]
    # somma le diverse scale
    for col in colonne_da_sommare:
        df[f'somma_{col[0]+1}_{col[1]}'] = df.iloc[:, col[0]:col[1]].sum(axis=1)
    # prendo solo le PSI
    colonne_nuove = df.filter(regex='somma_')
    colonne_nuove = colonne_nuove.div(4)
    # rinomina le colonne
    colonne_nuove.rename(columns={'somma_1_4': 'SOC', 'somma_5_8': 'AC', 'somma_9_12': 'PC', 'somma_13_16': 'RC'}, inplace=True)
    colonne_nuove['robot_type'] = type_filter

    return colonne_nuove

# per ogni modalità apre il file, somma le varie scale e fa la media
superficial = process_data('Superficial Deception')
external = process_data('External Deception')
hidden = process_data('Hidden Deception')

# concatena le modalità per salvarle su file
data = pd.concat([superficial, external, hidden])

# variabili PSI
variables = ['SOC', 'AC', 'PC', 'RC']

# dizionario da scrivere su file
anova_results = {'Variable': [], 
                 'Mean Superficial': [], 'Std Superficial': [],
                 'Mean External': [], 'Std External': [],
                 'Mean Hidden': [], 'Std Hidden': [],
                 'F-Statistic': [], 'P-Value': []}

# test
for variable in variables:
    # verifica se la distribuzione è normale
    _, p_shapiro_superficial = shapiro(superficial[variable])
    _, p_shapiro_external = shapiro(external[variable])
    _, p_shapiro_hidden = shapiro(hidden[variable])

    # omogenità varianze
    _, p_bartlett = stats.bartlett(superficial[variable], external[variable], hidden[variable])

    # stampa 
    print(variable, "\n\t", p_shapiro_superficial, "\n\t", p_shapiro_external, "\n\t", p_shapiro_hidden, 
                    "\n\t", p_bartlett, "\n")
 
    # esegui anova se le assunzioni sono soddisfatte
    if p_shapiro_superficial > 0.05 or p_shapiro_external > 0.05 or p_shapiro_hidden > 0.05 or p_bartlett > 0.05:
        f_statistic, p_value = f_oneway(superficial[variable], external[variable], hidden[variable])
        # dizionario per excel
        anova_results['Variable'].append(variable)
        anova_results['Mean Superficial'].append(superficial[variable].mean())
        anova_results['Std Superficial'].append(superficial[variable].std())
        anova_results['Mean External'].append(external[variable].mean())
        anova_results['Std External'].append(external[variable].std())
        anova_results['Mean Hidden'].append(hidden[variable].mean())
        anova_results['Std Hidden'].append(hidden[variable].std())
        anova_results['F-Statistic'].append(f_statistic)
        anova_results['P-Value'].append(p_value)
        # Plot
        fig, ax = plt.subplots()
        sns.violinplot(x='robot_type', y=variable, data=data)
        ax.set_xlabel("Condition")
        ax.set_ylabel("Value")
        plt.title(variable)
        fig.savefig("violin_" + variable + ".pdf")
        plt.close(fig)
    else:
        print("Impossible apply ANOVA!")
        sys.exit()

# crea il dataframe 
anova_df = pd.DataFrame(anova_results)
# stampo il dataframe
print(anova_df)

# post-hoc test
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
with pd.ExcelWriter('PSI_results.xlsx') as writer:  
    data.to_excel(writer, sheet_name='PSI_scales')
    anova_df.round(4).to_excel(writer, sheet_name='PSI_anova')
    tukey_df.to_excel(writer, sheet_name='PSI_tukey')
    