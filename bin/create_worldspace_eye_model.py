
# Modules
import numpy as np
import matplotlib.pyplot as plt
from os import path
import sys
from urllib import request

# Classes
from mpl_toolkits.mplot3d import Axes3D

# Functions
from rdp import rdp
from scipy.io import loadmat

def cartesian_to_spherical(vertices):
    hxy = np.hypot(vertices[:,0], vertices[:,2])

    azimuth = (np.pi / 2.0) + np.arctan2(vertices[:,2], vertices[:,0])
    elevation = np.arctan2(vertices[:,1], hxy)
    radius = np.hypot(hxy, vertices[:,1])

    return np.vstack((azimuth, -elevation, radius))

# Read bee ID from command line if present
BEE_ID = 6 if len(sys.argv) == 1 else int(sys.argv[1])

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

# Read the bounding volume of eye
eye_volume = data["eyeVolSize"][0][BEE_ID][0]

# Read the indices of the voxels that make up the surface of the eye
eye_surface_ind = data["EyeFrontSurfInds"][0][BEE_ID]

# Unravel these into 3D coordinates
eye_surface_coord = np.unravel_index(eye_surface_ind, eye_volume, order="F")

# Read indices of lower-resolution 'interpolation points'
eye_interpolation_points = data["interpPoints"][0][BEE_ID]

#eye_hex_area = data["interpFacetArea"][0][BEE_ID]

# Read intersection and border points of eye projected onto world-space sphere
eye_sphere_intersect = data["sphereIntersect"][0][BEE_ID]
eye_sphere_border = data["sphereIntersectBorders"][0][BEE_ID]

# Convert eye sphere border points to spherical coordinates
eye_border_spherical = cartesian_to_spherical(eye_sphere_border)
assert np.allclose(eye_border_spherical[2], 1.0)

# **HACK** sort by angle from arbitrary point so they're going in a sane order
eye_border_sort = np.arctan2(eye_border_spherical[1] - 1.0, eye_border_spherical[0] - 1.0)
eye_sort = np.argsort(eye_border_sort)
eye_border_spherical = eye_border_spherical[:,eye_sort]

eye_border_spherical = rdp(eye_border_spherical[:2].transpose(), epsilon=0.1)

# Write eye border ro file
np.degrees(eye_border_spherical).astype(np.float64).tofile(path.join(antworld_resource_directory, "world_eye_border_%s.bin" % bee_name))

# Plot eye surface at interpolation points
eye_fig = plt.figure()
eye_axis = eye_fig.add_subplot(111, projection="3d")
eye_axis.scatter(eye_surface_coord[0][eye_interpolation_points],
                 eye_surface_coord[1][eye_interpolation_points],
                 eye_surface_coord[2][eye_interpolation_points])
eye_axis.set_xlabel("X [uM]")
eye_axis.set_ylabel("Y [uM]")
eye_axis.set_zlabel("Z [uM]")

# Plot points on eye and its border projected onto reference unit sphere
sphere_fig = plt.figure()
sphere_axis = sphere_fig.add_subplot(111, projection="3d")
sphere_axis.scatter(eye_sphere_intersect[:,0], eye_sphere_intersect[:,1], eye_sphere_intersect[:,2])
sphere_axis.scatter(eye_sphere_border[:,0], eye_sphere_border[:,1], eye_sphere_border[:,2])
sphere_axis.set_xlabel("X")
sphere_axis.set_ylabel("Y")
sphere_axis.set_zlabel("Z")


pelt_fig, pelt_axis = plt.subplots()
pelt_axis.plot(eye_border_spherical[:,0], eye_border_spherical[:,1])
pelt_axis.set_xlabel("Azimuth [radians]")
pelt_axis.set_ylabel("Elevation [radians]")

plt.show()

