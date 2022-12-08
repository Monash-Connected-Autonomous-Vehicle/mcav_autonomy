#!/bin/sh

# This is only here to fix some errors that were happening with sudo and networking
# The warning was "sudo: unable to resolve host : Name or service not known"
# Put dynamic hostname into /etc/hosts to remove sudo warnings:
sudo sh -c "echo \"127.0.0.1	`hostname`.localdomain	`hostname`\" >> /etc/hosts" 2> /dev/null


# Start tmux session
session="mcav"
tmux new-session -d -s $session

tmux split-window -h
tmux send-keys './autonomy_launch/scripts/2.sh'
tmux split-window -v
tmux send-keys './autonomy_launch/scripts/3.sh'
tmux split-window -v -t 1
tmux send-keys './autonomy_launch/scripts/4.sh'
tmux select-pane -t 1
tmux send-keys './autonomy_launch/scripts/1.sh'

tmux attach-session -d -t $session