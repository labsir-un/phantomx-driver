## Lab install tools

## 1- Install python

- **Instal Python** from. [Download](https://www.python.org/downloads/)

### 2- Install ROS2
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
cd ..
```
build and source the project 
```bash
colcon build --symlink-install && source install/setup.bash
```

Con esto terminamos la primera session!
