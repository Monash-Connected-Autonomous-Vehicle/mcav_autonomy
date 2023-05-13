#!/bin/bash
# Script for building and working in a docker container

run_without_gpu()
{
    docker run -e DISPLAY -e TERM \
        --privileged \
        -v "/dev:/dev:rw" \
        -v "/lib/modules:/lib/modules:rw" \
        -v "$(pwd):/home/mcav/mcav_ws/src/mcav_autonomy:rw" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --net=host \
        --name $CONTAINER_NAME \
        --rm \
        -it $IMAGE_NAME bash -c /home/mcav/mcav_ws/src/mcav_autonomy/docker/tmux-start.sh
        #-d $IMAGE_NAME /usr/bin/tail -f /dev/null # was testing this to see if we can keep container running in background
	# --entrypoint /ros_entrypoint.sh \
}

build_image() 
{
    echo "Building docker image $IMAGE_NAME from $DOCKER_FILE"
    docker build . --platform linux/arm64 -t $IMAGE_NAME -f $DOCKER_FILE
}

CONTAINER_NAME=mcav_autonomy_px2
IMAGE_NAME=mcav_autonomy_px2
DOCKER_FILE=docker/Dockerfile.arm

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
    echo "Initialising container"
    run_without_gpu
    ;;
esac
