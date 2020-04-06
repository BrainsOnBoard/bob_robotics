# Ardin et al. Model of navigation
Published model of navigation using a spiking model of the ant mushroom body:
* Build simulator with CMake. Spikes can be recorded with ``-DRECORD_SPIKES=on`` option (e.g. ``cmake -DRECORD_SPIKES=on ..``) and synaptic weights with ``-DRECORD_TERMINAL_SYNAPSE_STATE=on`` option. If you don't have an NVIDIA GPU you should also specify the ``-DGENN_CPU_ONLY=on`` option.
* ``./ardin_mb ROUTE_FILE_NAME`` to run the simulator.
* SFML and GLEW are required - on Ubuntu can be installed with ``sudo apt-get install libglew-dev libsfml-dev``

If a route filename is passed to the simulator, the simulated ant will be trained on the route and then attempt to repeat it. Otherwise arrows keys allow manual ant exploration, snapshots can be trained with _space_ and matched with _enter_. The visualisation in this example is a C++ port of the Matlab code publically avaiable at [here](http://www.insectvision.org/walking-insects/antnavigationchallenge). Pressing _w_ performs a random walk and pressing _v_ after training calculates a vector field showing the best heading direction at each point in a grid surrounding the route.

Ardin, P., Peng, F., Mangan, M., Lagogiannis, K., & Webb, B. (2016). Using an Insect Mushroom Body Circuit to Encode Route Memory in Complex Natural Environments. PLoS Computational Biology, 12(2), 1–22. https://doi.org/10.1371/journal.pcbi.1004683
