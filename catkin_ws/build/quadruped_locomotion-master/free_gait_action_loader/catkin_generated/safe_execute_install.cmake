execute_process(COMMAND "/home/d/catkin_ws/build/quadruped_locomotion-master/free_gait_action_loader/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/d/catkin_ws/build/quadruped_locomotion-master/free_gait_action_loader/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
