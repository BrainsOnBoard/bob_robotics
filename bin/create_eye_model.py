# Modules
import numpy as np
import matplotlib.pyplot as plt
import sys
from urllib import request

# Classes
from descartes import PolygonPatch
from os import path

# Functions
from alphashape import alphashape
from scipy.io import loadmat
from scipy.optimize import least_squares

def cartesian_to_spherical(vertices):
    x = -vertices[:,0]
    y = -vertices[:,1]
    z = -vertices[:,2]

    hxz = np.hypot(x, z)

    radius = np.hypot(hxz, y)
    elevation = np.arctan2(y, hxz)
    azimuth = np.arctan2(z, x)

    return np.vstack((np.degrees(azimuth), np.degrees(elevation), radius))

def distance_to_line(p, x, y, z, nx, ny, nz):
    # Calculate dot product between normal and vector from vertex to x, y, z
    xs_dot_d = ((p[0] - x) * nx) + ((p[1] - y) * ny) + ((p[2] - z) * nz)

    # Calculate projection along direction of
    proj = [x + (xs_dot_d * nx),
            y + (xs_dot_d * ny),
            z + (xs_dot_d * nz)]

    # Return squared distance
    return ((proj[0] - p[0]) ** 2) + ((proj[1] - p[1]) ** 2) + ((proj[2] - p[2]) ** 2)

def calc_centre(vertices, vertex_normals, max_x):
    # Minimise distance between centre of the sphere and lines projected inwards along vertex normals from vertices
    return least_squares(distance_to_line, [max_x, 0.0, 0.0], method="dogbox",
                         bounds=([max_x, -np.inf, -np.inf], [0, np.inf, np.inf]),
                         args=(vertices[0], vertices[1], vertices[2],
                               vertex_normals[:,0], vertex_normals[:,1], vertex_normals[:,2]))["x"]

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


data = loadmat(path.join(antworld_resource_directory, "DataForPlots.mat"))["fullBeeData"]
bee_name = data["BeeID"][0][BEE_ID][0]
print("Bee name: %s" % bee_name)

# Read the surface voxels for original eye, transformed into beespace
eye_surface_coord = data["EyeFrontSurfSubsTrans"][0][BEE_ID]

# If the original eye was the left one, flip surface data
if data["LeftEyeOriginal"][0][BEE_ID][0]:
    print("FLIPPING!")
    eye_surface_coord[:,0] = -eye_surface_coord[:,0]

# Read indices of lower-resolution 'interpolation points'
eye_interpolation_inds = data["interpPoints"][0][BEE_ID]

# Use these to extract the beespace positions of eye surface voxels
eye_interpolation_points = np.vstack((eye_surface_coord[eye_interpolation_inds,0].flatten(),
                                      eye_surface_coord[eye_interpolation_inds,1].flatten(),
                                      eye_surface_coord[eye_interpolation_inds,2].flatten()))
# Get the border normals and indices
# **NOTE** these seem pretty dubious in general hence the fact we calculate our own alphashape - hopefully good enough for fitting
eye_border_normals = data["borderNormals"][0][BEE_ID]
eye_border_inds = data["borderInds"][0][BEE_ID]

# Extract border points
eye_border_points = np.vstack((eye_surface_coord[eye_border_inds,0].flatten(),
                               eye_surface_coord[eye_border_inds,1].flatten(),
                               eye_surface_coord[eye_border_inds,2].flatten()))

# Fit eye centre to point behind eye where border normals would intersect
# **TODO** this is slightly conservative, could fit a plane to 'back' of eye and rotate so we can operate from there
max_x = np.amax(eye_surface_coord[:,0])
eye_centre = calc_centre(eye_border_points, eye_border_normals, max_x)
print("Eye centre found at (%f, %f, %f)" % (eye_centre[0], eye_centre[1], eye_centre[2]))

# On this basis, convert eye interpolation points to spherical coordinates
eye_interpolation_spherical = cartesian_to_spherical(eye_interpolation_points.transpose() - eye_centre)

# Calculate alpha shape around eye
eye_alpha_shape = alphashape(eye_interpolation_spherical[:2].transpose(), 0.05)

# Write out to file
np.asarray(eye_alpha_shape.exterior.coords, dtype=np.float64).tofile(path.join(antworld_resource_directory, "eye_border_%s.bin" % bee_name))

# Plot eye FOV in spherical coordinates with alpha shape
fig, axis = plt.subplots()
axis.set_aspect("equal")
axis.scatter(eye_interpolation_spherical[0], eye_interpolation_spherical[1], s=1)
axis.add_patch(PolygonPatch(eye_alpha_shape, alpha=0.2))
axis.set_xlabel("Azimuth [degrees]")
axis.set_ylabel("Elevation [degrees]")

plt.show()

