# PhantomX Driver
This repo includes the basic packges for using a phantomX pincher arm with ROS2 (humble).
Includes: 
- `phantom_control`: For ros2_control integration. Custom topic-based HW plugin implementation for coppelia control. 
- `phantom_coppelia`: CoppeliaSim scene bringup templates. Ready to use with phantom_control
- `phantom_description`: URDF/Xacro files for robot states. 


## Getting started 
### Install ROS2
Install ROS2 humble and have it running. Here are some options depending in the OS. 
- **Ubuntu Jammy 22.04 LTS** from debian ource. [ros2 docs install tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **MacOS**: Use mamba and _robostack_ for installing the binaries for ros-humble. [robostack tutorial](https://robostack.github.io/GettingStarted.html)
- **Windows**: Use WSL for using a Linux Kernel and install ros as in Ubuntu OS

> Rember to source the ros workspace

### Workspace config
Clone this repo and configure the workspace.
```bash
mkdir phantom_ws && cd phantom_ws
mkdir src && cd src
git clone https://github.com/labsir-un/phantomx-driver.git
cd ../.. 
```
build and source the project 
```bash
colcon build --symlink-install && source install/setup.bash
```

## ros_control example 
Run the bringup `phantom_control` launchfile
```bash
ros2 launch phantom_control bringup_launch.py
```
You should be seeing the coppelia scene and rviz2 running up. Under the hood, ros2_control  is handling the phantom joints_states readings with a `joint_state_broadcaster` and a PositionController is commanding their joint config. 


<img width="70%" src="https://github.com/labsir-un/phantomx-driver/blob/main/docs/ros_control_example.gif"/> 





### Author 
Andrés Morales Martínez (amoralesma@unal.edu.co) - Mechatronics Engineer Student 

---
This packages were made for LabSir (Laboratorios de Sistemas Inteligentes Robotizados) 
