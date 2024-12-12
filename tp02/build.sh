#!/bin/bash

mkdir -p bin

part=part2

g++ -Wall -O2 -D_REENTRANT "src/$part.cpp" src/preprocess.cpp src/processing_threads.cpp src/utils.cpp -o "bin/$part" -lpthread -lrt
sudo "./bin/$part"
