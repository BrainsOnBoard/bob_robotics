# Robot data collection protocol

Here is a draft of what could be a checklist of things to do before
collecting data with the robot. This is just something off the top of my
head, so feel free to change it around however you like!

Extra things to bring:

-   An extra set of charged AA batteries (ideally in a battery case for
    easy testing)

    -   Some of the rechargeable ones Norbert has may well be buggered,
        so we should be buying more of these in any case. Also I think
        you can check if batteries are broken with a multimeter...

<!-- -->

-   An extra charged RC car battery

    -   Norbert's spare is broken, so he'll need to buy another

<!-- -->

-   The GPS base station

    -   We shouldn't need it (the GPS module should be able to access
        the subscription service over 3g), but it would be useful to
        have as a backup

<!-- -->

-   Small screwdrivers

-   A portable multimeter

    -   I bought one for exactly this reason at some point -- I'll have
        a look for it

Checks to do beforehand:

-   Check the AA battery packs with multimeter

-   Check the RC car batteries with multimeter

-   Run the data collection program and drive robot around garden or
    wherever

    -   This will tell us if any of the hardware isn't working

<!-- -->

-   Check the output of the data collection program

    -   I can write a tool to help with this, but you essentially just
        want to check that all the sensors are giving sensible readings
        and that all the data has been recorded (e.g. if you ran the
        test for 30s, do you have 30s' worth of data?) Atm, the sensors
        are:

        -   Camera

        -   GPS

        -   IMU

        -   A copy of the steering and speed commands sent to the
            robot's motors

I'm also proposing that we have the following rules on the software side
for any code we use for data collection in future:

-   The code should be committed to git and uploaded to github
    (otherwise how can we repeat the experiment in future?). We can save
    the git hash along with the data so we know which version was used

-   Code really should have been reviewed in a PR and merged into the
    main branch before we use it to get data for a publication (this
    means that it will have been reviewed by at least one other person
    and automated tests will be run on it). If it's not good enough to
    merge, it's really not good enough to collect data with.

I’ve just added loads more automated tests for our Github repo, so we
really want to take advantage of that infrastructure (it’s already found
a bunch of small issues with existing code).
