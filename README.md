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


To run
========

1/ Make sure that your robot library can be load by the component.
This means that your LD_LIBRARY_PATH should include the
library where the robot library is stored.

For instance, the HRP-2 robot controller library is located at:
/opt/openrobots/lib/libsot-hrp2-controller.so

Then the following line can be added in your .bashrc
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/openrobots/lib

2/ You also should make sure that your robot python scripts
for the Stack of Tasks are available in the PYTHONPATH.

For instance the HRP-2 python scripts are located at:
/opt/openrobots/lib/python2.6/site-packages/dynamic_graph/

Then the following line can be added in your .bashrc
PYTHON_PATH=$PYTHON_PATH:/opt/openrobots/lib/python2.6/site-packages/


Simple settings
===============
Make sure than ROS is running in a terminal by launching:
roscore

Go to the component directory
cd _build-RELEASE/components/

Launch the component:
./rtc-stack-of-tasks-comp

Activate the component:
cd $SRCDIR/components
./test-act-sot.sh



