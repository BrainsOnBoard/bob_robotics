## Perfect memory example
To build this example, you will need python headers and numpy installed.

The image set used for the example can be generated with the ``ant_world_db_creator`` tool. Just run:
```sh
# From bob_robotics
cd tools/ant_world_db_creator
mkdir build
cd build
cmake ..
make -j$(nproc)
../ant_world_db_creator ../../../resources/antworld/ant1_route1.bin
```
