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
        -v "$(pwd):/home/mcav/mcav_ws/src/mcav_autonomy:rw" \
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
        -v "$(pwd):/home/mcav/mcav_ws/src/mcav_autonomy:rw" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--net=host \
        --name $CONTAINER_NAME \
        --entrypoint /ros_entrypoint.sh \
        -d $IMAGE_NAME /usr/bin/tail -f /dev/null
        #--net=host \
        #-p 8765:8765 \
}

build_image() 
{
    echo "Building docker image $IMAGE_NAME from $DOCKER_FILE"
    docker build . --platform $PLATFORM -t $IMAGE_NAME -f $DOCKER_FILE
}

DISABLE_GPU=false
# Loop through all arguments, check if the flag was passed to disable GPU detection
for arg in "$@"
do
    case "$arg" in
    "--disable-gpu")
        DISABLE_GPU=true
        ;;
    esac
done

# Check for nvidia-container-runtime and set variables for GPU/no GPU accordingly
if [[ $(docker info | grep Runtimes) =~ nvidia ]] && [[ "$DISABLE_GPU" == false ]] ; then # computer has nvidia-container-runtime, use it for GPU support
    echo "GPU available"
    CONTAINER_NAME=mcav_autonomy_gpu
    IMAGE_NAME=ghcr.io/monash-connected-autonomous-vehicle/mcav_autonomy_gpu:main
    DOCKER_FILE=docker/Dockerfile.gpu
    GPU_AVAILABLE=true
    PLATFORM=linux/amd64
else # no nvidia-container-runtime
    echo "GPU not available. Install nvidia container runtime to enable support."
    GPU_AVAILABLE=false
    # Determine if running on ARM (jetson/M1) device or x86 (regular Intel computer)
    if [ "$(uname -m)" = "aarch64" ] || [ "$(uname -m)" = "arm64" ]; then # arm
        CONTAINER_NAME=mcav_autonomy_arm
        IMAGE_NAME=ghcr.io/monash-connected-autonomous-vehicle/mcav_autonomy_arm:main
        DOCKER_FILE=docker/Dockerfile.arm
        PLATFORM=linux/arm64
    else # x86_64
        echo "not arm"
        CONTAINER_NAME=mcav_autonomy_cpu
        IMAGE_NAME=mcav_autonomy_cpu
        IMAGE_NAME=ghcr.io/monash-connected-autonomous-vehicle/mcav_autonomy_cpu:main
        DOCKER_FILE=docker/Dockerfile.cpu
        PLATFORM=linux/amd64
    fi
fi


case "$1" in
"build")
    build_image
    ;;
"pull")
    docker pull $IMAGE_NAME
    ;;
"rm")
    if [[ $2 ]] ; then
        NAME_POSTFIX=$2
        CONTAINER_NAME="${CONTAINER_NAME}_${NAME_POSTFIX}"
    fi
    docker rm -f $CONTAINER_NAME
    echo "Removed container"
    ;;
"--help")
    echo "Usage: docker/run.sh [command] [name]
Available commands:
    run.sh [-n name] [--disable-gpu]
        Attach a new terminal to the container (pulling/building, creating and starting it if necessary).
        If supplied, the container will be given a specific name.
    run.sh build [--disable-gpu]
        Build a new image from the Dockerfile in the current directory.
    run.sh rm [-n name] [--disable-gpu]
        Remove the container.
        If name is supplied, it will remove the container with that name
    The "--disable-gpu" flag disables the automatic NVIDIA GPU detection, so that you can force the CPU only image/container.
    run.sh --help
        Show this help message    
    "
    ;;
*) # Attach a new terminal to the container (pulling, creating and starting it if necessary)
    if [[ $1 ]] ; then
        CONTAINER_NAME=$1
    fi

    if [ -z "$(docker images -f reference=$IMAGE_NAME -q)" ]; then # if the image does not yet exist, pull it
        if [ "$GPU_AVAILABLE" = true ] ; then
            build_image
        else
            build_image
        fi
    fi
    if [ -z "$(docker ps -qa -f name=$CONTAINER_NAME)" ]; then # if container has not yet been created, create it
        if [ "$GPU_AVAILABLE" = true ] ; then # computer has nvidia-container-runtime, use it for GPU support
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
