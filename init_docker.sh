#!/bin/bash

MODE=$1

RI_PATH="/media/carlo/42E0143574F95C22/ri/robotica/src/robotica_inteligente"

# If mode is not defined, print error message
if [ "$MODE" != "container" ] && [ "$MODE" != "object" ] && [ "$MODE" != "move" ] && [ "$MODE" != "all" ] && [ "$MODE" != "detection" ] && [ "$MODE" != "drive" ] && [ "$MODE" != "teleop" ]; then
    echo "Error: Mode not defined. Please, use 'container', 'object', 'move', 'detection', 'drive', 'teleop ' or 'all'"
    exit 1
fi

# MODE "container" or "all": Run container
if [ "$MODE" == "container" ] || [ "$MODE" == "all" ]; then
    gnome-terminal -- bash -c "xhost +local:; docker run --shm-size=1g --privileged --ulimit memlock=-1 --ulimit stack=67108864 --rm -it --net=host -e DISPLAY=:1 --user=root -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev:/dev --name robotica_container --gpus all --cpuset-cpus=0-3 -v $RI_PATH:/home/docker/catkin_ws/src/robotica_inteligente robotica-inteligente bash -c 'source devel/setup.bash; roslaunch robotica_inteligente load_scene.launch'"
fi

# MODE "teleop" or "all": Run ros launch
if [ "$MODE" == "teleop" ] || [ "$MODE" == "all" ]; then
    gnome-terminal -- bash -c "xhost +local:; 
        docker exec -it robotica_container /bin/bash -c 'source devel/setup.bash;
        apt-get install pip python3.8-tk -y; 
        pip install mediapipe==0.10.9; 
        roslaunch robotica_inteligente teleop_navigation.launch'"
fi

# MODE "object" or "all": Run camera
if [ "$MODE" == "object" ] || [ "$MODE" == "all" ]; then
    sleep 10
    gnome-terminal -- bash -c "xhost +local:; docker exec -it robotica_container /bin/bash -c 'source devel/setup.bash; rosrun robotica_inteligente object_localization.py'"
fi


# MODE "move" or "all": Move robot
if [ "$MODE" == "move" ] || [ "$MODE" == "all" ]; then
    gnome-terminal -- bash -c "xhost +local:; docker exec -it robotica_container /bin/bash -c 'source devel/setup.bash; rosrun robotica_inteligente move_robot_arm.py'"
fi

# MODE "detection" or "all": Run detection
if [ "$MODE" == "detection" ] || [ "$MODE" == "all" ]; then
    gnome-terminal -- bash -c "xhost +local:; docker exec -it robotica_container /bin/bash -c 'source devel/setup.bash; rosrun robotica_inteligente obstacle_detection.py'"
fi

# MODE "drive" or "all": Run drive
if [ "$MODE" == "drive" ] || [ "$MODE" == "all" ]; then
    gnome-terminal -- bash -c "xhost +local:; docker exec -it robotica_container /bin/bash -c 'source devel/setup.bash; rosrun robotica_inteligente blue_navigation.py'"
fi
