#!/bin/bash

# check if the first argument is cpp
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <source_file.cpp> [<executable_name>]"
    exit 1
fi

# define source and excecutable file
source_file="$1"
executable_name="${2:-${source_file%.cpp}}" # if not executable name given take the same as source file 

# check if source file exsits
if [ ! -f "$source_file" ]; then
    echo "Error: $source_file not found!"
    exit 1
fi

# Compile g++
g++ "$source_file" -Iinclude -o "$executable_name" `pkg-config --cflags --libs opencv4`

# check if it workes
if [ $? -eq 0 ]; then
    echo "Compilation successful! Executable created: $executable_name"
else
    echo "Compilation failed."
    exit 1
fi
