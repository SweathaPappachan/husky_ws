# CMake generated Testfile for 
# Source directory: /home/sweatha/husky_ws/src/husky/husky_gazebo
# Build directory: /home/sweatha/husky_ws/build/husky_gazebo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_husky_gazebo_roslaunch-check_launch "/home/sweatha/husky_ws/build/husky_gazebo/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/sweatha/husky_ws/build/husky_gazebo/test_results/husky_gazebo/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/sweatha/husky_ws/build/husky_gazebo/test_results/husky_gazebo" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/sweatha/husky_ws/build/husky_gazebo/test_results/husky_gazebo/roslaunch-check_launch.xml\" \"/home/sweatha/husky_ws/src/husky/husky_gazebo/launch\" ")
set_tests_properties(_ctest_husky_gazebo_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/sweatha/husky_ws/src/husky/husky_gazebo/CMakeLists.txt;8;roslaunch_add_file_check;/home/sweatha/husky_ws/src/husky/husky_gazebo/CMakeLists.txt;0;")
subdirs("gtest")
