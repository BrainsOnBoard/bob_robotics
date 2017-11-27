import csv
import matplotlib.pyplot as plt
import numpy as np

def get_csv_columns(csv_file, headers=True):
    # Create reader
    reader = csv.reader(csv_file, delimiter=",")

    # Skip headers if required
    if headers:
        reader.next()

    # Read columns and return
    return zip(*reader)

def get_column_safe(data, column, dtype):
    if column < len(data):
        return np.asarray(data[column], dtype=dtype)
    else:
        return []

with open("active_neurons.csv", "rb") as active_neurons_file:
    # Read data and zip into columns
    active_neurons_columns = get_csv_columns(active_neurons_file)

    # Convert CSV columns to numpy
    num_pn = get_column_safe(active_neurons_columns, 0, int)
    num_kc = get_column_safe(active_neurons_columns, 1, int)

    fig, axes = plt.subplots(2)

    axes[0].hist(num_pn, bins=range(0, 361, 10), normed=True)
    axes[0].set_title("PN")
    axes[0].set_xlabel("Num active in trial")

    axes[1].hist(num_kc, bins=range(np.amin(num_kc), np.amax(num_kc), 10), normed=True)
    axes[1].set_title("KC")
    axes[1].set_xlabel("Num active in trial")

    plt.show()