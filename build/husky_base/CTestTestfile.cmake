# CMake generated Testfile for 
# Source directory: /home/sweatha/husky_ws/src/husky/husky_base
# Build directory: /home/sweatha/husky_ws/build/husky_base
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_husky_base_roslaunch-check_launch "/home/sweatha/husky_ws/build/husky_base/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/sweatha/husky_ws/build/husky_base/test_results/husky_base/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/sweatha/husky_ws/build/husky_base/test_results/husky_base" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/sweatha/husky_ws/build/husky_base/test_results/husky_base/roslaunch-check_launch.xml\" \"/home/sweatha/husky_ws/src/husky/husky_base/launch\" ")
set_tests_properties(_ctest_husky_base_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/sweatha/husky_ws/src/husky/husky_base/CMakeLists.txt;46;roslaunch_add_file_check;/home/sweatha/husky_ws/src/husky/husky_base/CMakeLists.txt;0;")
subdirs("gtest")
