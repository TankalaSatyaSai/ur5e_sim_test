# Test 

rosdep install -i --from-paths src --ignore-src -r -y 

colcon build --symlink-install --packages-select ur5e ur5e_description ur5e_gazebo ur5e_moveit_config ur5e_system_tests ur5e_bringup ur5e_moveit_demos ur5e_mtc_demos; source install/setup.bash 