import pandas as pd
from path import Path

import csv
import os
import time
import json
import datetime
import math

ids = []
mod = []
turns = []
time_play = []
avg_time_until_match = []
number_of_rc_suggestions = []
number_of_card_suggestion = []
total_number_of_suggestions = []
number_of_suggestion_for_first_card = []
number_of_suggestions_followed = []
number_of_card_suggestion_not_followed = []
number_of_row_column_suggestion_not_followed = []
number_of_suggestions_for_first_card_not_followed = []
suggestion_first_card_with_match_in_next_turn = []
wrong_suggestions = []  # not all wrong suggestions, just those the user should have noticed 
                        #i.e the robot suggest a wrong card and user click another card and found a pair or the user click the card suggested and doesn't match


def get_turn_number(df):
    """
    get number of turns necessary to end a game for each user
    """
    
    tmp = df.loc[df['game_ended'] == True]      
    turns.append(tmp['turn_number'].iloc[0])


def get_time(df):
    # avg time to find a pair
    tmp = df.loc[df['match'] == True]
    tmp = tmp['time_until_match'].tolist()
    avg = (str(datetime.timedelta(seconds=math.trunc(sum(map(lambda f: int(f[0])*60 + int(f[1]), map(lambda f: f.split(':'), tmp)))/len(tmp)))))
    avg_time_until_match.append(avg)
    # time to finish the game
    mysum = datetime.timedelta()
    for i in tmp:
        (m, s) = i.split(':')
        d = datetime.timedelta(minutes=int(m), seconds=int(s))
        mysum += d
    time_play.append(str(mysum))
    

def calculate_suggestions(df):
    tmp = df.loc[df['suggestion_type'] != 'none']
    tmp = tmp['suggestion_type'].tolist()
    total_number_of_suggestions.append(len(tmp))

    tmp = df.loc[df['suggestion_type'] == 'card']
    tmp = tmp['suggestion_type'].tolist()
    number_of_card_suggestion.append(len(tmp))

    tmp = df.loc[(df['suggestion_type'] == 'row') | (df['suggestion_type'] == 'column')]
    tmp = tmp['suggestion_type'].tolist()
    number_of_rc_suggestions.append(len(tmp))

    tmp = df.loc[(df['turn_number'] % 2 != 0) & (df['suggestion_type'] != 'none')]
    tmp = tmp['suggestion_type'].tolist()
    number_of_suggestion_for_first_card.append(len(tmp))

    tmp = df.loc[(df['suggestion_type'] == 'card') & (df['turn_number'] % 2 == 0) & \
         (((df['position_clicked'] == df['position_suggested']) & (df['match'] == False)) | ((df['position_clicked'] != df['position_suggested']) & (df['match'] == True)))]
    wrong_suggestions.append(len(tmp))


def followed_suggestions(df):
    suggestions_followed = 0

    tmp = df.loc[(df['suggestion_type'] == 'card') & (df['position_clicked'] == df['position_suggested'])]
    suggestions_followed = len(tmp)

    tmp = df.loc[(df['suggestion_type'] == 'row') & (df['position_clicked'].str[1] == df['position_suggested'])]
    suggestions_followed += len(tmp)

    tmp = df.loc[(df['suggestion_type'] == 'column') & (df['position_clicked'].str[4] == df['position_suggested'])]
    suggestions_followed += len(tmp)

    number_of_suggestions_followed.append(suggestions_followed)

    # check if has followed the suggestion for first card and then if a pair has been found
    tmp = df.loc[(df['suggestion_type'] == 'card') & (df['position_clicked'] == df['position_suggested']) & (df['turn_number'] % 2 != 0) & (df['match'].shift(-1) == True)]
    suggestion_first_card_with_match_in_next_turn.append(len(tmp))
    

def suggestions_not_followed(df):
    tmp = df.loc[(df['suggestion_type'] == 'card') & (df['position_clicked'] != df['position_suggested'])]
    number_of_card_suggestion_not_followed.append(len(tmp))

    row_col_not_followed = 0
    tmp = df.loc[(df['suggestion_type'] == 'row') & (df['position_clicked'].str[1] != df['position_suggested'])]
    row_col_not_followed += len(tmp)
    tmp = df.loc[(df['suggestion_type'] == 'column') & (df['position_clicked'].str[4] != df['position_suggested'])]
    row_col_not_followed += len(tmp)
    number_of_row_column_suggestion_not_followed.append(row_col_not_followed)

    tmp = df.loc[(df['suggestion_type'] == 'card') & (df['position_clicked'] != df['position_suggested']) & (df['turn_number'] % 2 != 0)]
    number_of_suggestions_for_first_card_not_followed.append(len(tmp))

# get all csv 
path ="..\data\\"
df_append = pd.DataFrame()
for root, dirs, files in sorted(os.walk(path)):
    for file in files:
        if(file.endswith(".csv")):
            print(os.path.join(root,file))
            # one data frame for file
            df = pd.read_csv(os.path.join(root,file), sep=';')
            print(df)

            # get id and mod
            ids.append(df['id_player'].iloc[0])
            mod.append(df['experiment_condition'].iloc[0])

            # get turn number
            get_turn_number(df)

            # get time
            get_time(df)

            # get number of suggestion
            calculate_suggestions(df)

            # check if user followed the suggestion
            followed_suggestions(df)

            # get all suggestions that user has not followed
            suggestions_not_followed(df)

csv_struct = {
    "id": ids, "experiment_condition": mod, "turns": turns, "time to finish": time_play,
    "average time to find a pair": avg_time_until_match, "number of suggestions": total_number_of_suggestions,
    "number of card suggestions": number_of_card_suggestion, "number of row/col suggestions": number_of_rc_suggestions,
    "number of suggestions for first card": number_of_suggestion_for_first_card,
    "number of suggestions followed": number_of_suggestions_followed,
    "number of card suggestions not followed": number_of_card_suggestion_not_followed,
    "number of row/col suggestions not followed": number_of_row_column_suggestion_not_followed,
    "number of suggestions for first card not followed": number_of_suggestions_for_first_card_not_followed,
    "suggestions for the first card that let to a match": suggestion_first_card_with_match_in_next_turn,
    "wrong suggestions": wrong_suggestions
}

# write on excel the results
keys = csv_struct.keys()

# create dataframe
df_final = pd.DataFrame(csv_struct)
# order dataframe by id
df_final['id'] = df_final['id'].astype(int)
df_final.sort_values(by='id', inplace=True)
# handle time
df_final['time to finish'] = pd.to_timedelta(df_final['time to finish'])
df_final['time to finish'] = df_final['time to finish'].astype(str)
df_final['time to finish'] = df_final['time to finish'].apply(lambda x: str(x).split()[2]) # remove 0 days

# save it on excel
with pd.ExcelWriter("results.xlsx") as writer:
    df_final.to_excel(writer, index=False, float_format="%.2f")
