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

with open("terminal_synaptic_state.csv", "rb") as terminal_synaptic_state_file:
    # Read data and zip into columns
    terminal_synaptic_state_columns = get_csv_columns(terminal_synaptic_state_file)

    # Convert CSV columns to numpy
    weights = get_column_safe(terminal_synaptic_state_columns, 0, float)
    eligibility = get_column_safe(terminal_synaptic_state_columns, 1, float)

    fig, axes = plt.subplots(2)

    axes[0].hist(weights, bins=20)
    axes[0].set_title("Weight")

    axes[1].hist(eligibility, bins=20)
    axes[1].set_title("Eligibility")

    plt.show()