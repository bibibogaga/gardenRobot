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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/qt_ros_ui/qt_ros/qt_create"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/yoga/my_work/catkin_yoga/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/yoga/my_work/catkin_yoga/install/lib/python2.7/dist-packages:/home/yoga/my_work/catkin_yoga/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/yoga/my_work/catkin_yoga/build" \
    "/usr/bin/python2" \
    "/home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/qt_ros_ui/qt_ros/qt_create/setup.py" \
     \
    build --build-base "/home/yoga/my_work/catkin_yoga/build/TX2_StereoSLAM-master/src/qt_ros_ui/qt_ros/qt_create" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/yoga/my_work/catkin_yoga/install" --install-scripts="/home/yoga/my_work/catkin_yoga/install/bin"
