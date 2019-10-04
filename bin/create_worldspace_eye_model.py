
# Modules
import numpy as np
import matplotlib.pyplot as plt
from os import path
import sys
from urllib import request

# Classes
from descartes import PolygonPatch
from mpl_toolkits.mplot3d import Axes3D

# Functions
from alphashape import alphashape
from scipy.io import loadmat

def cartesian_to_spherical(vertices):
    hxy = np.hypot(vertices[:,0], vertices[:,2])

    azimuth = (np.pi / 2.0) + np.arctan2(vertices[:,2], vertices[:,0])
    elevation = np.arctan2(vertices[:,1], hxy)
    radius = np.hypot(hxy, vertices[:,1])

    return np.vstack((np.degrees(azimuth), -np.degrees(elevation), radius))

# Read bee ID from command line if present
BEE_ID = 6 if len(sys.argv) < 2 else int(sys.argv[1])
ALPHA = 0.05 if len(sys.argv) < 3 else float(sys.argv[2])

bin_directory = path.dirname(path.abspath(__file__))
antworld_resource_directory = path.join(bin_directory, "..", "resources", "antworld")


# If data isn't present, download from Dryad
if not path.exists(path.join(antworld_resource_directory, "DataForPlots.mat")):
    print("Downloading data from Dryad")
    request.urlretrieve("https://datadryad.org/stash/downloads/file_stream/18013",
                        path.join(antworld_resource_directory, "DataForPlots.mat"))

# Load data file
data = loadmat(path.join(antworld_resource_directory, "DataForPlots.mat"))["fullBeeData"]

bee_name = data["BeeID"][0][BEE_ID][0]
print("Bee name: %s" % bee_name)

# Read intersection points of eye projected onto world-space sphere
eye_sphere_intersect = data["sphereIntersect"][0][BEE_ID]

# Convert eye sphere points to spherical coordinates
eye_spherical = cartesian_to_spherical(eye_sphere_intersect)
assert np.allclose(eye_spherical[2], 1.0)

# Add mirrored copy for other eye
eye_spherical_mirrored = np.copy(eye_spherical)
eye_spherical_mirrored[0] = -eye_spherical[0]
eye_spherical = np.hstack((eye_spherical, eye_spherical_mirrored))

# Calculate alpha shape around eye
eye_alpha_shape = alphashape(eye_spherical[:2].transpose(), ALPHA)

# Save world eye border file
np.asarray(eye_alpha_shape.exterior.coords, dtype=np.float64).tofile(path.join(antworld_resource_directory, "world_eye_border_%s.bin" % bee_name))

# Plot eye FOV in spherical coordinates with alpha shape
fig, axis = plt.subplots()
axis.scatter(eye_spherical[0], eye_spherical[1], s=1)
axis.add_patch(PolygonPatch(eye_alpha_shape, alpha=0.2))
axis.axvline(0.0, linestyle="--", color="gray")
axis.set_xlabel("Azimuth [degrees]")
axis.set_ylabel("Elevation [degrees]")

plt.show()

