#!/bin/sh

item=item1
item=item3

mkdir -p bin

# -lm for <math.h> sqrt

# items 1-4, 6
gcc -Wall -O2 -D_REENTRANT $item.c jitter.c utils.c func.o -o bin/$item -lpthread -lrt -lm

# Item 5
#gcc -Wall -O2 -D_REENTRANT $item.c jitter.c utils.c func2.c -o bin/$item -lpthread -lrt -lm

sudo ./bin/$item
