# Image database recorder for RC car
Two programs can be built for collecting robot image databases from this directory: ``dataset_recorder`` and ``auto_dataset_recorder``, which automatically drives along a specified route. There is also an ``auto_dataset_recorder_simulated`` program for testing purposes.

You can choose the program to be built by passing the relevant arguments to CMake, e.g.:
```
cmake -DTARGET=auto_dataset_recorder
```
(If not specified, ``dataset_recorder`` is built by default.)

## ``dataset_recorder``
This program takes a single argument which is the amount of time to run data collection for in seconds. The robot can be driven with the remote control during this time.

## ``auto_datset_recorder``
This program takes a path to an existing image database as an argument (e.g. ``./auto_datset_recorder 20210817_171511``). The car should then drive along the same route recording images until either an error occurs or the route has been completed. The program can be terminated at any time by pressing Ctrl+C (which will also stop the robot moving).

Note that before the data collection begins, the robot will automatically drive forwards for a bit to compute its GPS heading.
