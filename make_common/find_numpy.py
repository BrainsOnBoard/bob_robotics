from os import path
from numpy import __file__ as numpyloc

# Get numpy directory
numpy_dir = path.dirname(numpyloc)

# Print the result of joining this to core and include
print(path.join(numpy_dir, "core", "include"))

