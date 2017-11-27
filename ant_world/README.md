# Stone et al. Model of path integration
Published model of navigation using a spiking model of the ant mushroom body:
* ``genn-buildmodel.sh model.cc`` to build network model
* ``make`` to build simulator. Spikes can be recorded with ``RECORD_SPIKES=1`` option and synaptic weights with ``RECORD_TERMINAL_SYNAPSE_STATE=1`` option.
* ``./ant_word ROUTE_FILE_NAME`` to run the simulator.

If a route filename is passed to the simulator, simulated ant will be trained on the route and then attempt to repeat it. Otherwise arrows keys allow manual ant exploration, snapshots can be trained with space and matched with enter. The visualisation in this example is a C++ port of the Matlab code publically avaiable at www.insectvision.org/walking-insects/antnavigationchallenge.

Ardin, P., Peng, F., Mangan, M., Lagogiannis, K., & Webb, B. (2016). Using an Insect Mushroom Body Circuit to Encode Route Memory in Complex Natural Environments. PLoS Computational Biology, 12(2), 1–22. https://doi.org/10.1371/journal.pcbi.1004683
