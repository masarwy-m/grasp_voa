import csv
import os
import statistics
import matplotlib.pyplot as plt
from matplotlib.table import Table

if __name__ == '__main__':
    angles = {}
    files = os.listdir('../obs_func_eval_res')
    csv_files = [os.path.join('../obs_func_eval_res', file) for file in files if file.endswith('csv')]
    for file in csv_files:
        with open(file, 'r') as csv_file:
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:
                if row[0] not in angles:
                    angles[row[0]] = []
                angles[row[0]].append((row[1], row[2]))
    res = []
    for angle in angles:
        if len(angles[angle]) <= 3:
            continue
        differences = [float(x) - float(y) for x, y in angles[angle]]
        # Calculate the mean and variance
        formatted_mean = "%.4f" % statistics.mean(differences)
        formatted_var = "%.4f" % statistics.variance(differences)
        res.append((angle, formatted_mean, formatted_var))

    with open('exp1.csv', 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerows(res)
