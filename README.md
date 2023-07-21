# SeekCamera Wrapper

Install seekcamera-sdk with

    sudo dpkg -i library/seekthermal-sdk-dev-4.3.1.6_amd64.deb

Build ROS2 package

Launch with

    ros2 launch seek_thermal_ros thermal_publisher_launch.py

*Tested on Ubuntu 22.04 with ROS2 Humble

Change colorPalette using args: **colorPalette:=value ** replace value with options mentioned below!

    Available options:
        - WHITE_HOT
        - BLACK_HOT
        - SPECTRA
        - PRISM
        - TYRIAN -- Default
        - IRON
        - AMBER
        - HI
        - GREEN

Change rotationValue using args: **--ros-args -p rotationValue:=value ** replace value with options mentioned below!

    Available options:
        - 0
        - 90 -- Default
        - 180
        - 270
        - 360

Sample image

![Sample RVIZ2](extras/images/sample_rviz2.png?raw=true "Sample RVIZ2")