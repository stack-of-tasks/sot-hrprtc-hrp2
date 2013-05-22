RTC component to encapsulate the SoT on HRP2
===========================================

This RTC component will load a dynamic library containing all the control
algorithms to generate motion for a humanoid robot.

Prerequisite
============
We assume that
* your machine is running ubuntu-10.04,
* ros-electric-desktop-full is installed in /opt/ros/electric (via apt-get),
* ros-electric-ros-realtime is installed in /opt/ros/electric,
* ros-electric-pr2-mechanism is installed in /opt/ros/electric,
* environment variable DEVEL_DIR should contain an existing directory.
* environment variable PKG_CONFIG_PATH should contain /opt/grx/lib/pkgconfig

The following packages should be installed in $DEVEL_DIR/install
* jrl-mathtools,
* jrl-mal,
* abstract-robot-dynamics,
* jrl-dynamics,
* jrl-walkgen,
* dynamic-graph,
* dynamic-graph-python,
* sot-core,
* sot-dynamics
* sot-patterngenerator,
* sot-hrp2.

The following ROS stacks should be installed in $DEVEL_DIR/stacks
* hrp2
* redundant_manipulator_control

To install
==========

        mkdir _build-RELEASE
        cd _build-RELEASE
        cmake -DCMAKE_INSTALL_PREFIX=$DEVEL_DIR/install -DCMAKE_BUILD_TYPE=RELEASE -DROBOT=$ROBOT ..
        make
        sudo chmod 777 /opt/grx/lib
        make install

where $ROBOT is HRP2LAAS or HRP2JRL.

Setting the environment variables for the component alone
=========================================================

1.  Make sure that your robot library can be loaded by the component.
    For that, your LD_LIBRARY_PATH should include $DEVEL_DIR/install/lib/plugin 
    and $DEVEL_DIR/install/lib.
    Check that libsot-hrp2-14-controller.so (or libsot-hrp2-10-controller.so) is
    in this latter directory.

2.  Make sure that $DEVEL_DIR/install/lib/python2.6/site-packages is in your
    PYTHONPATH to access SoT scripts.

Setting the environment variables for and the component inside a hrpsys graph
=========================================================================

At the end of file /opt/grx/$ROBOT/bin/config.sh, you should source the
following config.sh

        $DEVEL_DIR/config.sh

Be aware that the RTC component resets environment variables. Therefore, the
above DEVEL_DIR should be expanded by hand.

The above file should contain the following lines

        export ROS_ROOT=/opt/ros/electric/ros
        export PATH=$ROS_ROOT/bin:$PATH
        export PYTHONPATH=$PYTHONPATH:$ROS_ROOT/core/roslib/src:/opt/grx/HRP2LAAS/script
        export ROS_MASTER_URI=http://localhost:11311

Running the component (with GRX software)
=========================================

1.  Make sure than ROS is running in a terminal by launching:

        roscore

2.  if you are doing simulation launch the simulator
(Grx 3.1 simulator for HRP2) and the simulation,

3.  launch /opt/grx/$ROBOT/script/guisot.py,

4.  Click on setup rt-system,

5.  rosrun dynamic_graph_bridge run_command

6.  initialize your control graph

7.  click on Start SoT


Introspection tools for the RTC component
=========================================

        rtcat /localhost/sot.rtc 
will give you the state of the Stack of Task RTC component.

If you want to see the command:

        rtprint /localhost/sot.rtc:qRef

To check the format of the port:

        rtcat -l /localhost/sot.rtc:qRef


