#!/bin/sh

# This is only here to fix some errors that were happening with sudo and networking
# The warning was "sudo: unable to resolve host : Name or service not known"
# Put dynamic hostname into /etc/hosts to remove sudo warnings:
sudo sh -c "echo \"127.0.0.1	`hostname`.localdomain	`hostname`\" >> /etc/hosts" 2> /dev/null


# Start tmux session
session="mcav"
tmux new-session -d -s $session

tmux attach-session -d -t $session
