import csv
import os
import statistics
import matplotlib.pyplot as plt
from matplotlib.table import Table

if __name__ == '__main__':
    angles = {}
    files = os.listdir('../results/sprayflask/different_pov')
    csv_files = [os.path.join('../results/sprayflask/different_pov', file) for file in files if
                 not file.startswith('data')]
    table = {}
    for data_file in csv_files:
        differences = []
        with open(data_file, 'r') as csv_file:
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:
                differences.append(float(row[1]) - float(row[2]))
        file = data_file.split('/')[-1]
        if int(file[0]) not in table.keys():
            table[int(file[0])] = {}
        formatted_mean = "%.4f" % statistics.mean(differences)
        formatted_var = "%.4f" % statistics.variance(differences)
        table[int(file[0])][int(file[2])] = (formatted_mean, formatted_var)
    rows = [[''] + list(table[1].keys())]
    for key, row in table.items():
        rows.append([key] + list(table[key].values()))
    with open('../results/sprayflask/different_pov/sum.csv', 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerows(rows)
