#!/bin/bash

mkdir -p bin

part=part1

g++ -Wall -O2 -D_REENTRANT "$part.cpp" preprocess.cpp utils.c -o "bin/$part" -lpthread -lrt
sudo "./bin/$part"
