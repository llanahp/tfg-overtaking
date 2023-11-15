import os
import csv

def write_csv(obs, timestamp):
    results_path = f"./scenarios/SMARTS/poses"

    if not os.path.exists(results_path):
        print("Create results path folder: ", results_path)
        os.makedirs(results_path)

    with open(f'{results_path}/poses_{timestamp}.csv', 'w', newline='') as file:
        writer = csv.writer(file, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)

        for key in obs.keys():
                for num_obs in range(len(obs[key])):
                    writer.writerow([key, 
                                    num_obs, 
                                    obs[key][num_obs][0],
                                    obs[key][num_obs][1],
                                    obs[key][num_obs][2]])