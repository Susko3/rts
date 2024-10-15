#!/bin/bash

build_and_run() {
    local item=$1
    local source_file=$2
    local func_file=$3
    mkdir -p bin
    gcc -Wall -O2 -D_REENTRANT -DITEM=$item $source_file jitter.c utils.c $func_file -o bin/$item -lpthread -lrt
    sudo ./bin/$item
}

PS3="Select the work item (1-7): "

select item in Item1 Item3 Item4_AlternativeA Item4_AlternativeB Item4_AlternativeB_Invert Item5 Item6; do
    source_file=item3.c
    func_file=func.o

    case $item in
    Item1)
        source_file=item1.c
        ;;
    Item5)
        func_file=func2.c
        ;;
    esac

    build_and_run $item $source_file $func_file
done
