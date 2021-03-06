# Import modules
import itertools
import re
import matplotlib.pyplot as plt
import numpy as np

def load_vicon_csv(filename):
    with open(filename, "rb") as csv_file:
        # Skip headers
        csv_file.readline()
        csv_file.readline()

        # Read header row
        header_row = csv_file.readline().split(",")

        # Count objects present in header
        num_objects = (len(header_row) - 2) / 6
        print("%u objects tracked" % num_objects)

        # Parse object name columns
        obj_names = [re.match(r"Global Angle (.*):(.*)", col).group(1)
                     for col in header_row[2:-1:6]]

        # Build a dtype with named fields
        column_suffixes = ("_rx", "_ry", "_rz", "_x", "_y", "_z")
        dtype = [("frame", int), ("subframe", int)] + list(itertools.chain.from_iterable([(n + s, float) for s in column_suffixes]
                                                                                         for n in obj_names))

        # Parse raw data into numpy
        raw_data = np.genfromtxt(csv_file, dtype=dtype, delimiter=",", skip_header=2, filling_values=np.nan, invalid_raise=False)

        # Loop through objects
        data = {}
        for n in obj_names:
            # Stack together this object's columns of data
            object_data = np.vstack([raw_data["frame"]] + [raw_data[n + s] for s in column_suffixes])

            # Strip out any rows with NaN i.e. invalid valuesd
            data[n] = object_data[:,~np.isnan(object_data).any(axis=0)]

        return data

# Load Vicon data file
vicon_data = load_vicon_csv("pi.csv")
robot_data = vicon_data["norbot"]

outbound_end_frame = 981849 - 977162
outbound_mask = (robot_data[0,:] < outbound_end_frame)
inbound_mask = np.logical_not(outbound_mask)

fig, axis = plt.subplots()

axis.plot(robot_data[4,outbound_mask], robot_data[5,outbound_mask], color="red", label="Outbound")
axis.plot(robot_data[4,inbound_mask], robot_data[5,inbound_mask], color="blue", label="Inbound")
axis.set_xlabel("X")
axis.set_ylabel("Y")
axis.legend()

plt.show()