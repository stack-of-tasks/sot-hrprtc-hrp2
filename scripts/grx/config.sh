 #!/bin/sh
 source /opt/ros/electric/setup.bash
 ROS_WS_DIR=$HOME/devel/ros-unstable-2
 source $ROS_WS_DIR/setup.bash
 ROS_WS_DIR=$HOME/devel/ros-unstable-2
 export ROBOT="HRP2LAAS"
 export ROS_ROOT=/opt/ros/electric/ros
 export PATH=$ROS_ROOT/bin:$PATH
 export PYTHONPATH=$ROS_ROOT/core/roslib/src:$ROS_WS_DIR/install/lib/python2.6/site-packages/:$PYTHONPATH
 export PKG_CONFIG_PATH=$ROS_WS_DIR/install/lib/pkgconfig:/usr/local/lib/pkgconfig:/opt/grx/lib/pkgconfig
 export ROS_PACKAGE_PATH=$ROS_WS_DIR:$ROS_WS_DIR/stacks/hrp2:$ROS_WS_DIR/stacks/ethzasl_ptam:/opt/ros/electric/stacks:/opt/ros/electric/stacks/ros_realtime:$ROS_PACKAGE_PATH
 export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_WS_DIR/install/lib:$ROS_WS_DIR/install/lib/plugin
 export ROS_MASTER_URI=http://localhost:11311
 

