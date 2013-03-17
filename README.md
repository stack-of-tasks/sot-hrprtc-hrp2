RTC component to encapsulte the SoT 
===================================

This RTC component will load a dynamic library containing
all the control algorithms to generate motion for a humanoid robot.

To install
===================================

mkdir _build-RELEASE

cd _build-RELEASE

cmake -DCMAKE_BUILD_TYPE=RELEASE ..

make

make install

Setting the environment variables for the component alone
=========================================================


1/ Make sure that your robot library can be load by the component.
This means that your LD_LIBRARY_PATH should include the
library where the robot library is stored.

For instance, the HRP-2 robot controller library is located at:
/opt/openrobots/lib/libsot-hrp2-controller.so

Then the following line can be added in your shell starting file:
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/openrobots/lib

2/ You also should make sure that your robot python scripts
for the Stack of Tasks are available in the PYTHONPATH.

For instance the HRP-2 python scripts are located at:
/opt/openrobots/lib/python2.6/site-packages/dynamic_graph/

Then the following line can be added in your shell starting file:
PYTHON_PATH=$PYTHON_PATH:/opt/openrobots/lib/python2.6/site-packages/

Setting the environment variables for the component inside a hrpsys graph
=========================================================================

1/ Environment variables
We assume that you have a config file such as the one
given in the subdirectory:
scripts/grx/config.sh

It setup, in addition to the above variables, the ROS environment.

This file has to be included in the config.sh of your
robot, typically :
/opt/grx/YOUR_ROBOT/bin/config.sh

2/ Use guisot.py to launch the graph of RTC-component in
scripts/grx


Robot settings for the RTC-component
====================================

For the controller to load the proper library,
you should modify the sotinfo.py files and set:
1/ The name of the library in sot.libname
2/ The number of the robot DOFs
3/ The number of force sensors.

Here is an example for the HRP-2 robot.

sotConfig=[["sot.libname","libsot-hrp2-14-controller.so"],
           ["robot.nbdofs","36"],
           ["robot.nb_force_sensors","4"]]


Running the component (with GRX software)
=========================================

1/ Make sure than ROS is running in a terminal by launching:
roscore

2/ If you are doing simulation launch the simulator
(Grx 3.1 simulator for HRP2)

3/ launch guisot.py

TODO: See if this working outside GRX test-suite.


