#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/matrice/dji-GNC-ROS/src/Onboard-SDK-ROS/dji_sdk"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/matrice/dji-GNC-ROS/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/matrice/dji-GNC-ROS/install/lib/python2.7/dist-packages:/home/matrice/dji-GNC-ROS/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/matrice/dji-GNC-ROS/build" \
    "/usr/bin/python" \
    "/home/matrice/dji-GNC-ROS/src/Onboard-SDK-ROS/dji_sdk/setup.py" \
    build --build-base "/home/matrice/dji-GNC-ROS/build/Onboard-SDK-ROS/dji_sdk" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/matrice/dji-GNC-ROS/install" --install-scripts="/home/matrice/dji-GNC-ROS/install/bin"
