execute_process(COMMAND "/home/yoga/my_work/catkin_yoga/build/TX2_StereoSLAM-master/src/qt_ros_ui/qt_ros/qt_create/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/yoga/my_work/catkin_yoga/build/TX2_StereoSLAM-master/src/qt_ros_ui/qt_ros/qt_create/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
