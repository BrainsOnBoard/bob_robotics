# Ardin et al. Model of navigation
Published model of navigation using a spiking model of the ant mushroom body:
* ``genn-buildmodel.sh model.cc`` to build network model (``genn-buildmodel.sh -c model.cc`` if you don't have a NVIDIA GPU)
* ``make`` to build simulator. Spikes can be recorded with ``RECORD_SPIKES=1`` option and synaptic weights with ``RECORD_TERMINAL_SYNAPSE_STATE=1`` option. If you don't have an NVIDIA GPU you should also specify the ``CPU_ONLY=1`` option.
* ``./ant_word ROUTE_FILE_NAME`` to run the simulator.
* GLFW3 and GLEW are required - on Ubuntu can be installed with ``sudo apt-get install libglew-dev libglfw3-dev``

If a route filename is passed to the simulator, the simulated ant will be trained on the route and then attempt to repeat it. Otherwise arrows keys allow manual ant exploration, snapshots can be trained with _space_ and matched with _enter_. The visualisation in this example is a C++ port of the Matlab code publically avaiable at www.insectvision.org/walking-insects/antnavigationchallenge. Pressing _w_ performs a random walk and pressing _v_ after training calculates a vector field showing the best heading direction at each point in a grid surrounding the route.

Ardin, P., Peng, F., Mangan, M., Lagogiannis, K., & Webb, B. (2016). Using an Insect Mushroom Body Circuit to Encode Route Memory in Complex Natural Environments. PLoS Computational Biology, 12(2), 1–22. https://doi.org/10.1371/journal.pcbi.1004683
