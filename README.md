# VINS-FUSION Jumpstart


## Quickstart

Clone this repo, initialize submodules
```
git clone ...
git submodule update --init
```
(you might need --recursive but I dont think so)

Build the docker image
```
docker compose build
```

Run the docker image, make sure the camera is connected before you do this.
```
docker compose up &
```

use `docker exec -it <container_name> bash` to enter the container. 

Inside the docker, start a few terminals
```
vins_camera # to launch the realsense camera
vins_path   # to start the VINS-visual inertial odometry module
vins_loop   # to start the VINS loop closure
vins_rviz   # (optional) to launch the rviz window to debug vins-fusion
```

Launch the vicon bridge (optional):
```
roslaunch vicon_bridge vicon.launch 
```
Dont forget to set the IP address of the vicon computer in `catkin_ws/src/vicon_bridge/launch/vicon.launch`

If you want to align the vins world with the vicon world, before starting the vins commands, run
```
rosrun vins align.py
```
Launch octomap (optional):
In another terminal 
```
roslaunch vins octomap_mapping.launch 
```

