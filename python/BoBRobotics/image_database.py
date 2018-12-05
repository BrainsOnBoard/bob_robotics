import pyaml
import os
import pandas as pd

# Loads the metadata for the image database


def load_metadata(database_path):
    with open(os.path.join(database_path, 'database_metadata.yaml'), 'r') as fin:
        _, tail = fin.read().split('\n', 1)
        return pyaml.yaml.load(tail)['metadata']

# Gets a list of coordinates and image file paths for the database


def load(database_path):
    metadata = load_metadata(database_path)

    df = pd.read_csv(os.path.join(database_path, 'database_entries.csv'))
    df.columns = df.columns.str.strip()  # Trim whitespace from headings

    entries = dict()
    for _, row in df.iterrows():
        x, y, z, theta, i, j, k, filename = row[['X [mm]', 'Y [mm]', 'Z [mm]',
                                                 'Heading [degrees]',
                                                 'Grid X', 'Grid Y', 'Grid Z',
                                                 'Filename']]
        entries[(i, j, k)] = {'x': x, 'y': y, 'z': z, 'theta': theta,
                              'filepath': os.path.join(database_path, filename.strip())}

    return entries, metadata
