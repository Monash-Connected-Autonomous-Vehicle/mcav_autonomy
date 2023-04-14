#!/bin/bash
set -euox # stops the script if there are errors

./external/SD-VehicleInterface/can_setup.sh
docker/run.sh 
