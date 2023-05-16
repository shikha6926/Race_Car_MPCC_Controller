#!/bin/bash
# Get path to this file
SCRIPT=$(readlink -f "$0")
FILE_PATH=$(dirname "$SCRIPT")

# Create backup folder located in experiment folder
now=$(date +"%Y_%m_%d")
time=$(date +"%H_%M_%S")

# Find argument that follows _type= and use it to determin folder to store
# i.e. sim or real
# Little bit ugly of argument parsing
prefix=type=
record=0
for argument in "$@"
do
    if [[ $argument == type=* ]] 
    then
    TARGET_PATH=$FILE_PATH/../../../../rosbags/${now}/${time}
    fi

    if [[ $argument == record=* ]] 
    then
        record=$argument
    fi
done
mkdir -p $TARGET_PATH
mkdir -p $TARGET_PATH/config

# Copy all <configuration.yaml> files to backup folder
for cfg_path in "$@"
do
    if [[ $cfg_path == *.yaml ]] 
    then
    cp $cfg_path $TARGET_PATH/config # Backup all configuration files
    fi
done

# Regard all topics as a ros bag
if [[ $record == record=true ]] 
then
rosrun rosbag record -a -o $TARGET_PATH/
fi