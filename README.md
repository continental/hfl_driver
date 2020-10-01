# Continental's HFL110 ROS Driver
This package was designed to be a [Robotic Operating System (ROS)](https://index.ros.org/about/) driver for [Continental's 3D Flash Lidar products](https://www.continental-automotive.com/en-gl/Passenger-Cars/Autonomous-Mobility/Enablers/Lidars/3D-Flash-Lidar).

**Supported platforms/releases**:
| Platform                                                   | ROS Release                                                    |
| ---------------------------------------------------------- | -------------------------------------------------------------- |
| [Ubuntu 16.04 Bionic](https://releases.ubuntu.com/16.04.4/) | [ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu) |
| [Ubuntu 18.04 Bionic](https://releases.ubuntu.com/18.04/) | [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu) |
| [Ubuntu 20.04 Bionic](https://releases.ubuntu.com/20.04/) | [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) |

**License**: BSD Two Clause License

Please [review the source code documentation](https://continental.github.io/hfl_driver/index.html) for more details on how the project is structured.

## Quickstart

Install like any other ROS package:
```
sudo apt install ros-<ros-distro>-hfl-driver
```
**Note:** there may be a delay from when new code is available in this repository to when it will become available via apt.

## Install from source

First, make sure your system is supported and already has ROS installed (see table above)

Go ahead and clone this repository into your `catkin_ws`.
```
# url
git clone https://github.com/continental/hfl_driver.git
# ssh
git clone git@github.com:continental/hfl_driver.git
```
Read up on `catkin_ws` by [following this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

From your `catkin_ws` directory, use `rosdep` to install dependencies rosdep install hfl_driver:
```
rosdep install hfl_driver
```

Within your `catkin_ws` directory, go ahead and compile the code:
```
catkin_make
# with tests
catkin_make run_tests
```

After a successful compile, add the new HFL ROS packages to your environment:
```
echo "source <path/to/your/catkin_ws>/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

In two separate terminals, run the following commands.

Terminal 1:
```
roscore
```

Terminal 2:
```
roslaunch hfl_driver hfl110dcu.launch
```

[Parameters](http://wiki.ros.org/roslaunch/XML/arg) for hfl_driver launch file

| Parameter           | Description           | Default Values        |
| ------------------- | --------------------- |:---------------------:|
| camera_model        | HFL Model to launch   | hfl110dcu             |
| camera_version      | HFL Firmware version  | v1                    |
| camera_ip_address   | HFL IP address (IPv4) | 192.168.10.21         |
| frame_data_port     | HFL PCA Port          | 57410                 |
| computer_ip_address | Computer IPv4 Address | 192.168.10.5          |

**TIP**: check a launch files arguments before calling roslaunch to confirm you are passing the correct parameters.

**TIP**: If you cannot connect to the sensor, use [wireshark](https://www.wireshark.org/) or another network tool to see if you are receiving packets.

Be sure to check the [documentation website](https://continental.github.io/hfl_driver/index.html) for more information.

## CPP static code analysis

ROS also comes with static code analysis support, therefore in order to run it for the hfl_driver package, type:
```bash
catkin_make run_tests roslint_hfl_driver
```
This will output the errors and warnings on console.
If more info is required see [this](http://wiki.ros.org/roslint).

### Authors
Many have contributed to this project beyond just the people listed here.
Thank you to those who have answered any questions, emails or supported the project in other ways.
Without you none of this would have been possible.
- Gerardo Bravo
- Evan Flynn
