import pandas as pd
import numpy as np


def read_real_reading(config_file):
    df = pd.read_csv(config_file, header=None)

    result = np.ones((360,))
    for index, row in df.iterrows():
        degrees = int(row[0])
        float_value = row[1]
        result[degrees] = float_value
    return result


if __name__ == '__main__':
    config_f = '../results/endstop_holder/different_pov/1_2.csv'
    print(read_real_reading(config_f))
