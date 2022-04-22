#!/bin/bash
# Script for building and working in a docker container

attach_to_container() 
{
    # Allow docker windows to show on our current X Server
    xhost + >> /dev/null

    # Start the container in case it's stopped
    docker start $CONTAINER_NAME

    # Attach a terminal into the container
    exec docker exec -it $CONTAINER_NAME bash -c /home/mcav/mcav_ws/src/mcav_autonomy/docker/tmux-start.sh
}

run_with_gpu()
{
    docker run -e DISPLAY -e TERM \
        --privileged \
        -v "/dev:/dev:rw" \
        -v "$(pwd):/home/mcav/mcav_ws/src/minidrone:rw" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --runtime=nvidia \
        --net=host \
        --name $CONTAINER_NAME \
        --gpus all \
        --entrypoint /ros_entrypoint.sh \
        -d $IMAGE_NAME /usr/bin/tail -f /dev/null
}
run_without_gpu()
{
    docker run -e DISPLAY -e TERM \
        --privileged \
        -v "/dev:/dev:rw" \
        -v "$(pwd):/home/mcav/mcav_ws/src/minidrone:rw" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --net=host \
        --name $CONTAINER_NAME \
        --entrypoint /ros_entrypoint.sh \
        -d $IMAGE_NAME /usr/bin/tail -f /dev/null
}

build_image() 
{
    echo "Building docker image $IMAGE_NAME from $DOCKER_FILE"
    docker build . -t $IMAGE_NAME -f $DOCKER_FILE
}


# Can force usage of jetson container by running 'run.sh jetson' or 'run.sh build jetson'
if [ "$2" = "jetson" ] || [ "$1" = "jetson" ]; then
    force_jetson=1
else
    force_jetson=0
fi
# Determine if running on ARM (jetson) device or x86
if [ "$(uname -m)" = "aarch64" ] || [ $force_jetson -eq 1 ]; then # arm
    CONTAINER_NAME=minidrone_jetson
    IMAGE_NAME=minidrone_jetson
    DOCKER_FILE=docker/Dockerfile.jetson
else # x86_64
    CONTAINER_NAME=minidrone_x86
    IMAGE_NAME=minidrone_x86
    DOCKER_FILE=docker/Dockerfile
fi  

case "$1" in
"build")
    build_image
    ;;
"rm")
    docker rm -f $CONTAINER_NAME
    echo "Removed container"
    ;;
"--help")
    echo "Usage: docker/run.sh [command]
Available commands:
    run.sh
        Attach a new terminal to the container (pulling/building, creating and starting it if necessary)
    run.sh build
        Build a new image from the Dockerfile in the current directory
    run.sh rm
        Remove the current container
    run.sh --help
        Show this help message    
    "
    ;;
*) # Attach a new terminal to the container (building, creating and starting it if necessary)
    if [ -z "$(docker images -f reference=$IMAGE_NAME -q)" ]; then # if the image does not yet exist, build it
        build_image
    fi
    if [ -z "$(docker ps -qa -f name=$CONTAINER_NAME)" ]; then # if container has not yet been created, create it
        # Check for nvidia-container-runtime and run corresponding command
        if [[ $(docker info | grep Runtimes) =~ nvidia ]] ; then # computer has nvidia-container-runtime, use it for GPU support
            echo "Initialising with GPU support"
            run_with_gpu
        else # no nvidia-container-runtime
            echo "Initialising without GPU support"
            run_without_gpu
        fi  
    fi
    attach_to_container
    ;;
esac
