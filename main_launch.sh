#!/bin/bash
set -euox # stops the script if there are errors

./SD-VehicleInterface/can_setup.sh
docker/run.sh 
