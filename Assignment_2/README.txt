Pre-Requisites:
    - Install ROS
    - Install STDR
    - Copy all the contents of 'stdr_resources/maps' to the respective folder inside the stdr_resources package 
        (use 'roscd stdr_resources/maps')
    - Copy all the contents of 'stdr_resources/resources/robots' to the respective folder in the stdr_resources package
        (use 'roscd stdr_resources/resources/robots')

Compilation / Installation:
    - Execute 'catkin_make' in the root directory

Use:
    1) Initialize the STDR GUI + Map + Robot
        - Execute:
            - 'roslaunch stdr_launchers d_map_outside.launch' for robot outside single 'D' wall
            - 'roslaunch stdr_launchers d_map_inside.launch' for robot inside single 'D' wall
            - 'roslaunch stdr_launchers double_d_map.launch' for robot in double 'D' wall
    
    2) Make the robot detect and follow walls
        - Execute 'source devel/setup.sh'
        - Execute 'rosrun reactive_robot robot <robot_id> <laser_id>', where <robot_id> and <laser_id> are given by
            the STDR (usually robot0 and laser_0 but this values can change).
    
    3) Enjoy your basic reactive robot!