#!/bin/bash

gnome-terminal -- ./autonomy_launch/scripts/autoware_ai.sh

./SD-VehicleInterface/can_setup.sh
docker/run.sh 
