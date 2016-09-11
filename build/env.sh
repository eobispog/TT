#!/bin/sh


if [ $# -eq 0 ] ; then
    /bin/echo "Entering build environment at /home/eobispog/ros_workspace/ardrone_tutorials/build"
    . /home/eobispog/ros_workspace/ardrone_tutorials/build/setup.sh
    $SHELL -i
    /bin/echo "Exiting build environment at /home/eobispog/ros_workspace/ardrone_tutorials/build"
else
    . /home/eobispog/ros_workspace/ardrone_tutorials/build/setup.sh
    exec "$@"
fi


