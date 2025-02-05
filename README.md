ros2_sample
===================================================

Overview
--------------------------
This is a repository for ROS2 training.  
When cloning, please clone directly under ${HOME}

Quick Start
--------------------------
### Create WS  
1. Change the script's permissions   
```
bash ./scripts/create_ws.sh
```
2. Install pakage's  
```
cd scripts
./colcon_build_install.sh
```
3. Docker install  
```
./docker_install.sh
./ros_image_pull.sh
```
4. Docker build  
```
./docker_build.sh
```
5. Docker run  
```
./docker_run.sh
```
