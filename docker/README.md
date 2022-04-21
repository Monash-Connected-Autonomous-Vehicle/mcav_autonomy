# 🐋 mcav-docker
Contains Dockerfiles and a run script.

The `run.sh` script automatically determines whether a GPU is available and builds and runs the appropriate image and container.

The `mcav_autonomy` directory is mounted as a volume so it can be accessed from inside and outside the container.

Networking and GUIs should work as normal inside and outside.

## Requirements
Docker: `sudo apt-get install docker.io`

[Nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit) (For optional GPU support) 

## How to use

### First run
This will build the image if it doesn't exist and enter the container automatically.

- `cd mcav_autonomy`
- `docker/run.sh`

### Rebuild the image
- `docker/run.sh build`

### Create a new container from the most recently built image
- `docker/run.sh rm` (removes the existing container)
- `docker/run.sh`
